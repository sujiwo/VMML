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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImageDatabase.h"
#include "vmml/BaseFrame.h"
#include "ImagePipeline.h"
#include "ProgramOptions.h"
#include "vision_mapper/place_recognizer.h"


using namespace Vmml;
using namespace std;


ImageDatabase imageDb;
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		6000,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
		10);
Mapper::ImagePipeline *imagePipe;


bool PlaceRecognizerService(
	vision_mapper::place_recognizer::Request &request,
	vision_mapper::place_recognizer::Response &response)
{
/*
	cv_bridge::CvImagePtr imageReq = cv_bridge::toCvCopy(request.input, sensor_msgs::image_encodings::BGR8);

	cv::Vec3f
		weights(0.3333, 0.3333, 0.3333),
		sigmas(10, 10, 10);
	cv::Mat workImg = ImagePreprocessor::retinaHdr(imageReq->image, weights, sigmas, 128, 128, 1.0, 10);
*/
	cv::Mat workImg, mask;
	imagePipe->run(request.input, workImg, mask);
//	cv::resize(workImg, workImg, cv::Size(), 0.6666666667, 0.6666666666667);
//	workImg = ImagePreprocessor::autoAdjustGammaRGB(workImg);

	auto queryFrame = BaseFrame::create(workImg);
	queryFrame->computeFeatures(bFeats, mask);

	vector<vector<cv::DMatch>> featureMatches;
	imageDb.searchDescriptors(queryFrame->allDescriptors(), featureMatches, 2, 32);
	// Filter matches according to ratio test
	vector<cv::DMatch> realMatches;
	for (uint m=0; m<featureMatches.size(); m++) {
		if (featureMatches[m][0].distance < featureMatches[m][1].distance * 0.65)
			realMatches.push_back(featureMatches[m][0]);
	}

	vector<ImageMatch> imageMatches;
	imageDb.searchImages(queryFrame->allDescriptors(), realMatches, imageMatches);
	response.keyframeId.clear();
	for (int i=0; i<min(15, (int)imageMatches.size()); i++) {
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

	Vmml::Mapper::ProgramOptions progOptions;
	string mapPath;
	progOptions.addSimpleOptions("map-path", "Path to Map File", mapPath);
	progOptions.parseCommandLineArgs(argc, argv);
	imagePipe = &progOptions.getImagePipeline();

	cout << "Loading map... " << flush;
	imageDb.loadFromDisk(mapPath);
	cout << "Done\n";

	ros::ServiceServer placeRecognSrv = rosNode.advertiseService("place_recognizer", PlaceRecognizerService);
	cout << "Ready\n";

	ros::spin();
	return 0;
}
