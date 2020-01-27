/*
 * place_recognizer_srv.cpp
 *
 *  Created on: Jan 7, 2020
 *      Author: sujiwo
 */

/*
 * This service loads image index created from index_creator,
 * and answers to image query as keyframe number
 */

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImageDatabase.h"
#include "vmml/BaseFrame.h"
#include "vmml/ImagePreprocessor.h"
#include "vision_mapper/place_recognizer.h"


using namespace Vmml;
using namespace std;


ImageDatabase imageDb;
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		2000,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);


bool PlaceRecognizerService(
	vision_mapper::place_recognizer::Request &request,
	vision_mapper::place_recognizer::Response &response)
{
	cv_bridge::CvImagePtr imageReq = cv_bridge::toCvCopy(request.input, sensor_msgs::image_encodings::BGR8);

	cv::Mat workImg;
	cv::resize(imageReq->image, workImg, cv::Size(), 0.6666666667, 0.6666666666667);
	workImg = ImagePreprocessor::autoAdjustGammaRGB(workImg);
	cv::imwrite("/tmp/procx.png", workImg);

	auto queryFrame = BaseFrame::create(workImg);
	queryFrame->computeFeatures(bFeats);

	vector<vector<cv::DMatch>> featureMatches;
	imageDb.searchDescriptors(queryFrame->allDescriptors(), featureMatches, 2, 64);
	// Filter matches according to ratio test
	vector<cv::DMatch> realMatches;
	for (uint m=0; m<featureMatches.size(); m++) {
		if (featureMatches[m][0].distance < featureMatches[m][1].distance * 0.8)
			realMatches.push_back(featureMatches[m][0]);
	}

	vector<ImageMatch> imageMatches;
	imageDb.searchImages(queryFrame->allDescriptors(), realMatches, imageMatches);
	response.keyframeId.clear();
	for (int i=0; i<min(10, (int)imageMatches.size()); i++) {
		int bagRefId = imageDb.keyframeIdToBag.at(imageMatches[i].image_id);
		response.keyframeId.push_back(bagRefId);
//		cout << imageMatches[i].image_id << ' ' << imageMatches[i].score << endl;
	}

	return true;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "place_recognizer");
	ros::NodeHandle rosNode;

	imageDb.loadFromDisk(argv[1]);

	ros::ServiceServer placeRecognSrv = rosNode.advertiseService("place_recognizer", PlaceRecognizerService);
	cout << "Ready\n";

	ros::spin();
	return 0;
}
