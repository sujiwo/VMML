/*
 * test_imagedb.cpp
 *
 *  Created on: Dec 24, 2019
 *      Author: sujiwo
 */


#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "vmml/BaseFrame.h"
#include "vmml/ImageDatabase.h"


using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	ImageDatabase visDb;
	visDb.loadFromDisk(argv[1]);

	cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
			500,
			1.2,
			8,
			32,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			32,
			10);

	cv::Mat image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
	auto queryFrame = BaseFrame::create(image);
	queryFrame->computeFeatures(bFeats);

	vector<vector<cv::DMatch>> featureMatches;
	visDb.searchDescriptors(queryFrame->allDescriptors(), featureMatches, 2, 64);
	// Filter matches according to ratio test
	vector<cv::DMatch> realMatches;
	for (uint m=0; m<featureMatches.size(); m++) {
		if (featureMatches[m][0].distance < featureMatches[m][1].distance * 0.8)
			realMatches.push_back(featureMatches[m][0]);
	}

	vector<ImageMatch> imageMatches;
	visDb.searchImages(queryFrame->allDescriptors(), realMatches, imageMatches);

	for (int i=0; i<min(10, (int)imageMatches.size()); i++) {
		cout << imageMatches[i].image_id << ' ' << imageMatches[i].score << endl;
	}

	return 0;
}
