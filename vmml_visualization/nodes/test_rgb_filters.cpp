/*
 * test_rgb_filters.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <thread>
#include <memory>
#include <exception>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d.hpp>
#include <rosbag/bag.h>
#include <csignal>
#include "vmml/ImagePreprocessor.h"
#include "vmml/utilities.h"
#include "ImagePipeline.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"
#include "sift/sift.hpp"


using namespace std;
using cv::xfeatures2d::SIFT;


image_transport::Publisher imagePub1, imagePub2;
auto akzDetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE, 256, 3, 0.03f, 8);
auto orbDetector = cv::ORB::create(
		3000,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
		10);
auto siftDetector = SIFT::create(6000);
const float alpha = 0.3975;

Vmml::Mapper::ImagePipeline *imgPipe;

bool hasBreak = false;


void breakHandler(int sign)
{
	if (sign==SIGINT)
		hasBreak = true;
	cout << "Break is pressed\n";
}


cv::Mat imagePipelineRun (const cv::Mat &srcRgb)
{
	cv::Mat mask, imageReady;

	imgPipe->run(srcRgb, imageReady, mask);

	// Detector Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	orbDetector->detectAndCompute(
			imageReady,
		mask,
		kpList,
		descriptors);
	cv::drawKeypoints(imageReady, kpList, drawFrameKeypts, cv::Scalar(0,255,0));

	return drawFrameKeypts;
}


cv::Mat imagePipelineRun (const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgBgr = cv_bridge::toCvShare(imgMsg, "bgr8");
	return imagePipelineRun(imgBgr->image);
}


void doPublish (const cv::Mat &image)
{
	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = image;
	cvImg.header.stamp = ros::Time::now();

	imagePub1.publish(cvImg.toImageMsg());
}


void imageHandlerFunc(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgBgr = cv_bridge::toCvShare(imgMsg, "bgr8");
	cv::Mat drawKf = imagePipelineRun(imgBgr->image);

	return doPublish(drawKf);
}


void setupRosPublisher(Vmml::Mapper::RVizConnector &rosCtl, Vmml::Mapper::ProgramOptions &progOpts)
{
	auto imageTopic = progOpts.getImageTopic();

	image_transport::ImageTransport iTrans(*rosCtl.getNodeHandle());
	imagePub1 = iTrans.advertise(imageTopic+"/preprocess", 1);
}


void runFromRosNode (Vmml::Mapper::RVizConnector &rosCtl, Vmml::Mapper::ProgramOptions &progOpts)
{
	setupRosPublisher(rosCtl, progOpts);

	ros::Subscriber imgSub = rosCtl.getNodeHandle()->subscribe(progOpts.getImageTopic(), 1, imageHandlerFunc);
	ros::spin();
}


void runFromBagFile (Vmml::Mapper::RVizConnector &rosCtl, Vmml::Mapper::ProgramOptions &progOpts)
{
	if (rosCtl.isRosUsed())
		setupRosPublisher(rosCtl, progOpts);

	auto bagFile = progOpts.getImageBag();

	string outBagPathStr;
	float bagResample=-1, startTime=-1, stopTime=-1;
	try { outBagPathStr = progOpts.getOptionValue<string>("bag-output"); } catch (out_of_range &e) {}
	try { bagResample = progOpts.getOptionValue<float>("resample"); } catch (out_of_range &e) {}
	try { startTime = progOpts.getOptionValue<float>("start-time"); } catch (out_of_range &e) {}
	try { stopTime = progOpts.getOptionValue<float>("stop-time"); } catch (out_of_range &e) {}

	shared_ptr<rosbag::Bag> outputBagFd = nullptr;
	string imageOutTopic = progOpts.getImageTopic() + "/preprocess";

	if (outBagPathStr.empty()==false) {
		cout << "Output is to Bag file: " << outBagPathStr << endl;
		outputBagFd.reset(new rosbag::Bag(outBagPathStr, rosbag::BagMode::Write));
		if (outputBagFd->isOpen()==false) {
			cerr << "Unable to create bag file\n";
			exit(1);
		}
	}

	else {
		cout << "Output is to ROS\n";
	}

	bagFile->setTimeConstraint(startTime, stopTime);
	RandomAccessBag::DesampledMessageList targetFrame;
	if (bagResample!=-1) {
		bagFile->desample(bagResample, targetFrame);
	}
	else {
		targetFrame.resize(bagFile->size());
		for (uint i=0; i<bagFile->size(); ++i)
			targetFrame[i] = i;
	}

	for (uint i: targetFrame) {

		if (hasBreak==true)
			break;

		auto imageMsg = bagFile->getMessage(i);
		auto tMsg = bagFile->timeAt(i);
		auto imagePreprocess = imagePipelineRun(imageMsg);

		if (rosCtl.isRosUsed()) {
			doPublish(imagePreprocess);
		}

		if (outputBagFd!=nullptr) {
			cv_bridge::CvImage cvImg;
			cvImg.encoding = sensor_msgs::image_encodings::BGR8;
			cvImg.image = imagePreprocess;
			cvImg.header.stamp = imageMsg->header.stamp;
			outputBagFd->write(imageOutTopic, tMsg, cvImg.toImageMsg());
		}

		cout << i+1 << " / " << bagFile->size() << endl;
	}
}


int main(int argc, char *argv[])
{
	Vmml::Mapper::RVizConnector rosCtl(argc, argv, "test_rgb_filters");

	Vmml::Mapper::ProgramOptions progOpts;
	string outputBag;
	float resample, startTime, stopTime;

	progOpts.addSimpleOptions("bag-output", "Bag output", &outputBag);
	progOpts.addSimpleOptions("resample", "Rate for playing bag", &resample);
	progOpts.addSimpleOptions("start-time", "Process will start from x seconds", &startTime);
	progOpts.addSimpleOptions("stop-time", "Maximum seconds from start", &stopTime);

	progOpts.parseCommandLineArgs(argc, argv);
	imgPipe = &progOpts.getImagePipeline();
	imgPipe->doGammaCorrection = false;

	signal(SIGINT, breakHandler);

	if (progOpts.getBagPath().string().empty()==true) {
		cout << "Running using ROS\n";
		runFromRosNode(rosCtl, progOpts);
	}
	else {
		cout << "Running from Bag\n";
		runFromBagFile(rosCtl, progOpts);
	}

	return 0;
}
