/*
 * test_rgb_filters.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d.hpp>
#include "vmml/ImagePreprocessor.h"
#include "vmml/utilities.h"
#include "ImagePipeline.h"
#include "ProgramOptions.h"


using namespace std;


image_transport::Publisher imagePub1, imagePub2;
auto detector2 = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE, 256, 3, 0.03f, 8);
auto detector = cv::ORB::create(
		3000,
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


void imageHandler(const sensor_msgs::Image::ConstPtr &imgMsg)
{
	auto imgBgr = cv_bridge::toCvShare(imgMsg, "bgr8");
	cv::Mat mask, imageReady;

	imgPipe.run(imgMsg, imageReady, mask);

	// ORB Test
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors, drawFrameKeypts;
	detector->detectAndCompute(
		imageReady,
		mask,
		kpList,
		descriptors);
	cv::drawKeypoints(imageReady, kpList, drawFrameKeypts, cv::Scalar(0,255,0));
//	cerr << "# features: " << kpList.size() << endl;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.image = drawFrameKeypts;
	cvImg.header.stamp = ros::Time::now();

	imagePub1.publish(cvImg.toImageMsg());
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_rgb_filters");
	ros::NodeHandle mNode;

	Vmml::Mapper::ProgramOptions progOpts;
	string
		segnetModelPath,
		segnetWeightsPath,
		imageTopic;
	progOpts.addSimpleOptions("segnet-model", "Path to SegNet Model", segnetModelPath);
	progOpts.addSimpleOptions("segnet-weight", "Path to SegNet Weights", segnetWeightsPath);
	progOpts.parseCommandLineArgs(argc, argv);
	imageTopic = progOpts.getImageTopic();

	imgPipe.setSemanticSegmentation(segnetModelPath, segnetWeightsPath);

	image_transport::ImageTransport iTrans(mNode);
	imagePub1 = iTrans.advertise(imageTopic+"/preprocess", 1);

	ros::Subscriber imgSub = mNode.subscribe(imageTopic, 1, imageHandler);
	ros::spin();

	return 0;
}
