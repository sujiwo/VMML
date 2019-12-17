/*
 * index_creator.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: sujiwo
 */

#include <vector>
#include <string>
#include <iostream>
#include <rosbag/bag.h>
#include "ImageBag.h"
#include "VisionMap.h"
#include "Trajectory.h"
#include "TrajectoryGNSS.h"
#include "Matcher.h"
#include "ImageDatabase.h"
#include "utilities.h"


using namespace std;
using namespace Vmml;


// 5 cm/s
const float
linearVelocityThreshold = 0.05,
linearDistThreshold = 100.0,
linearIdleThreshold = 10.0,
imageSameThrScore = 0.15;

const uint
numberOfFeatures = 3000;


cv::Ptr<cv::DescriptorMatcher> bMatcher = cv::BFMatcher::create();
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		numberOfFeatures,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);


float compareAndScore(const BaseFrame &f1, const BaseFrame &f2)
{
	Matcher::PairList fr12Matches;
	Matcher::matchEpipolar(f1, f2, fr12Matches, bMatcher);
	return (float)fr12Matches.size() / (float)f1.numOfKeyPoints();
}


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag images(mybag, "/camera1/image_raw", 0.416666666667);
	uint width, height;
	images.getImageDimensions(width, height);

	cv::Mat queryImg = cv::imread("query-image.png", cv::IMREAD_GRAYSCALE);
	if (queryImg.empty())
		cerr << "Unable to open query image\n";

	CameraPinholeParams camera0(0, 0, 0, 0, width, height);

	auto trackGnss = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence");
	trackGnss.dump("gnss.csv");

	Trajectory trackImage;
	kfid curKf = 0;
	map<kfid, int> kfToFrameNum;

	ImageDatabase imageDb;

	auto imageAnchor = BaseFrame::create(images.at(0), camera0);
	imageAnchor->computeFeatures(bFeats);
	imageDb.addImage(curKf, imageAnchor->allKeypoints(), imageAnchor->allDescriptors());
	kfToFrameNum[curKf] = 0;

//	const int maxLim = images.size();
	const int maxLim = 2000;
	for (int i=1; i<maxLim; ++i) {
		auto curImage = BaseFrame::create(images.at(i), camera0);
		ptime imageTimestamp = images.timeAt(i).toBoost();
		curImage->computeFeatures(bFeats);

		if (curImage->numOfKeyPoints()<=10) {
			cout << "XXX!\n";
			continue;
		}

		imageDb.addImage2(i, curImage->allKeypoints(), curImage->allDescriptors());

		float comparisonScore = compareAndScore(*imageAnchor, *curImage);
		bool isKeyFrame=false;

		if (comparisonScore<=imageSameThrScore) {
			curKf += 1;
			imageAnchor = curImage;
			PoseStamped imagePose = trackGnss.at(imageTimestamp);
			trackImage.push_back(imagePose);
			isKeyFrame = true;

//			imageDb.addImage(curKf, curImage->allKeypoints(), curImage->allDescriptors());
			kfToFrameNum[curKf] = i;
		}

		cout << i+1 << " / " << maxLim << (isKeyFrame==true?"*":"") << endl;
	}

	trackGnss.dump("gnss.csv");
	trackImage.dump("images.csv");

	cout << "Done mapping\n";

	// Image search
	auto queryFrame = BaseFrame::create(queryImg, camera0);
	queryFrame->computeFeatures(bFeats);
	vector<vector<cv::DMatch>> featureMatchesFromIdx;

	// Searching the query descriptors against the features
	imageDb.searchDescriptors(queryFrame->allDescriptors(), featureMatchesFromIdx, 2, 64);

	// Filtering matches according to the ratio test
	vector<cv::DMatch> matches;
	for (unsigned m = 0; m < featureMatchesFromIdx.size(); m++) {
		if (featureMatchesFromIdx[m][0].distance < featureMatchesFromIdx[m][1].distance * 0.8) {
			matches.push_back(featureMatchesFromIdx[m][0]);
		}
	}

    vector<ImageMatch> image_matches;
    // We look for similar images according to the good matches found
    imageDb.searchImages(queryFrame->allDescriptors(), matches, image_matches);

    for (int i=0; i<min(5, (int)image_matches.size()); ++i) {
    	cout << image_matches[i].image_id << " : " << image_matches[i].score << endl;
    }
//    for (auto &imgMatch: image_matches) {
//    	cout << imgMatch.first << ": " << imgMatch.second << endl;
//    }
    cout << "# of images: " << imageDb.numImages() << endl;
    cout << "# of descriptors: " << imageDb.numDescriptors() << endl;

	return 0;
}
