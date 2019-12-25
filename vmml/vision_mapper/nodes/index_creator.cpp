/*
 * index_creator.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: sujiwo
 */

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <rosbag/bag.h>
#include "vmml/ImageBag.h"
#include "vmml/VisionMap.h"
#include "vmml/Trajectory.h"
#include "vmml/TrajectoryGNSS.h"
#include "vmml/Matcher.h"
#include "vmml/ImageDatabase.h"
#include "vmml/utilities.h"


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


template<typename K, typename V>
void writeCsv(const string &filepath, const map<K,V> &container)
{
	fstream fd(filepath, fstream::out|fstream::trunc);
	for(auto &fmp: container) {
		fd << fmp.first << ' ' << fmp.second << endl;
	}
	fd.close();
}


string selectTopicForGnssLocalization(const rosbag::Bag &bagsrc)
{
	auto topicList = RandomAccessBag::getTopicList(bagsrc);
	for (auto &topicPair: topicList) {
		if (topicPair.second=="nmea_msgs/Sentence" or topicPair.second=="sensor_msgs/NavSatFix")
			return topicPair.first;
	}
	throw runtime_error("There is no supported message type for GNSS Localization");
}


int main(int argc, char *argv[])
{
	cout << "Opening bag... ";
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());
	cout << "Done" << endl;

	ImageBag images(mybag, "/front_rgb/image_raw", 0.416666666667);
	uint width, height;
	images.getImageDimensions(width, height);

	cv::Mat queryImg = cv::imread("query-image.png", cv::IMREAD_GRAYSCALE);
	if (queryImg.empty())
		cerr << "Unable to open query image\n";

	CameraPinholeParams camera0(0, 0, 0, 0, width, height);

	auto gnssTopic = selectTopicForGnssLocalization(mybag);
	auto trackGnss = TrajectoryGNSS::fromRosBagSatFix(mybag, gnssTopic);
	trackGnss.dump("gnss.csv");

	Trajectory trackImage;
	kfid curKf = 0;
	map<kfid, int> kfToFrameNum;

	ImageDatabase imageDb;

	auto imageAnchor = BaseFrame::create(images.at(0), camera0);
	imageAnchor->computeFeatures(bFeats);
	imageDb.addImage(curKf, imageAnchor->allKeypoints(), imageAnchor->allDescriptors());
	kfToFrameNum[curKf] = 0;

	const int maxLim = images.size();
//	const int maxLim = 1000;
	for (int i=1; i<maxLim; ++i) {
		auto curImage = BaseFrame::create(images.at(i), camera0);
		ptime imageTimestamp = images.timeAt(i).toBoost();
		curImage->computeFeatures(bFeats);

		float comparisonScore = compareAndScore(*imageAnchor, *curImage);
		bool isKeyFrame=false;

		if (comparisonScore<=imageSameThrScore) {
			curKf += 1;
			imageAnchor = curImage;
			PoseStamped imagePose = trackGnss.at(imageTimestamp);
			trackImage.push_back(imagePose);
			isKeyFrame = true;

			imageDb.addImage2(curKf, curImage->allKeypoints(), curImage->allDescriptors());
			kfToFrameNum[curKf] = i;

			// put keyframe images for reference
			string imgName = "kf" + to_string(curKf) + ".png";
			cv::imwrite(imgName, curImage->getImage());
		}

		cout << i+1 << " / " << maxLim << (isKeyFrame==true?"*":"") << endl;
//		if (imageDb.numImages()==3) break;
	}

	trackImage.dump("images.csv");

	// write csv file for mapping from keyframe# -> bagframe#
	writeCsv("frames.csv", kfToFrameNum);

	cout << "Done mapping\n";

	imageDb.saveToDisk("imagedb-2000.dat");
	return 0;
}
