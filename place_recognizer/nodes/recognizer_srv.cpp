/*
 * recognizer_srv.cpp
 *
 *  Created on: Sep 29, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "IncrementalBoW.h"
#include "ProgramOptionParser.h"
#include "place_recognizer/place_recognizer.h"


using namespace std;


class RecognizerService
{
public:
RecognizerService(ros::NodeHandle &node, PrgOpt::ProgramOption &opt)
{
	binFeats = cv::ORB::create(
			opt.get<int>("numfeats", 6000),
			1.2,
			8,
			31,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			31,
			10);
	imagedb.loadFromDisk(opt.get<string>("mapfile", ""));
	placeRecognSrv = node.advertiseService("place_recognizer", &RecognizerService::service, this);
	cout << "Ready\n";
}

bool service(
	place_recognizer::place_recognizer::Request &request,
	place_recognizer::place_recognizer::Response &response)
{
	auto image = cv_bridge::toCvShare(request.input, nullptr, "bgr8")->image;
	std::vector<cv::KeyPoint> kpList;
	cv::Mat descriptors;
	binFeats->detectAndCompute(image, cv::Mat(), kpList, descriptors, false);

	vector<vector<cv::DMatch>> featureMatches;
	imagedb.searchDescriptors(descriptors, featureMatches, 2, 32);
	// Filter matches according to ratio test
	vector<cv::DMatch> realMatches;
	for (uint m=0; m<featureMatches.size(); m++) {
		if (featureMatches[m][0].distance < featureMatches[m][1].distance * 0.65)
			realMatches.push_back(featureMatches[m][0]);
	}

	vector<PlaceRecognizer::ImageMatch> imageMatches;
	imagedb.searchImages(descriptors, realMatches, imageMatches);
	response.keyframeId.clear();
	for (int i=0; i<min(15, (int)imageMatches.size()); i++) {
		response.keyframeId.push_back(imageMatches[i].image_id);
	}

	return true;
}

static
PrgOpt::ProgramOption
prepare_options()
{
	PrgOpt::ProgramOption opts;
	opts.addSimpleOptions("mapfile", "Map file input path");
	opts.addSimpleOptions<int>("numfeats", "Number of features from single image");
	return opts;
}

private:
	PlaceRecognizer::IncrementalBoW imagedb;
	ros::ServiceServer placeRecognSrv;
	cv::Ptr<cv::FeatureDetector> binFeats;
};


int main(int argc, char *argv[])
{
	auto opts = RecognizerService::prepare_options();
	opts.parseCommandLineArgs(argc, argv);

	ros::init(argc, argv, "place_recognizer");
	ros::NodeHandle node;

	RecognizerService srv(node, opts);
	ros::spin();

	return 0;
}
