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


using namespace std;


image_transport::Publisher imagePub1, imagePub2;
auto detector2 = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE, 256, 3, 0.03f, 8);
auto detector = cv::ORB::create(
		6000,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
		10);
const float alpha = 0.3975;

Vmml::Mapper::ImagePipeline imgPipe;

bool hasBreak = false;


void breakHandler(int sign)
{
	if (sign==SIGINT)
		hasBreak = true;
}


cv::Mat imagePipelineRun (const cv::Mat &srcRgb)
{
	cv::Mat mask, imageReady;

	imgPipe.run(srcRgb, imageReady, mask);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
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
	try { outBagPathStr = progOpts.getOptionValue<string>("bag-output"); }
	catch (out_of_range &e) {}

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

	for (uint i=0; i<bagFile->size(); ++i) {

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

		cout << i << " / " << bagFile->size() << endl;
	}
}


int main(int argc, char *argv[])
{
	Vmml::Mapper::RVizConnector rosCtl(argc, argv, "test_rgb_filters");

	Vmml::Mapper::ProgramOptions progOpts;
	string
		segnetModelPath,
		segnetWeightsPath,
		imageTopic,
		imageMask,
		outputBag;

	progOpts.addSimpleOptions("segnet-model", "Path to SegNet Model", segnetModelPath);
	progOpts.addSimpleOptions("segnet-weight", "Path to SegNet Weights", segnetWeightsPath);
	progOpts.addSimpleOptions("image-mask", "Path to Dashboard Mask", imageMask);
	progOpts.addSimpleOptions("bag-output", "Bag output", outputBag);

	progOpts.parseCommandLineArgs(argc, argv);
	imageTopic = progOpts.getImageTopic();

	imgPipe.setRetinex();
	imgPipe.setResizeFactor(progOpts.getImageResizeFactor());

	if (segnetModelPath.empty()==false and segnetWeightsPath.empty()==false)
		imgPipe.setSemanticSegmentation(segnetModelPath, segnetWeightsPath);
	if (imageMask.empty()==false)
		imgPipe.setFixedFeatureMask(imageMask);

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
