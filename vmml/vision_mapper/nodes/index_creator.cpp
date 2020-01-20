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
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;


// 5 cm/s
const float
linearVelocityThreshold = 0.05,
linearDistThreshold = 100.0,
linearIdleThreshold = 10.0,
imageSameThrScore = 0.15;

const uint
numberOfFeatures = 1500;


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


class IndexCreator
{
public:
IndexCreator(const CameraPinholeParams &c, uint numOfFeaturesReq=numberOfFeatures) :
	cameraPars(c)
{
	 bMatcher = cv::BFMatcher::create();
	 bFeats = cv::ORB::create(
		 numOfFeaturesReq,
			1.2,
			8,
			32,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			32,
			10);
}

void run(const ImageBag::Ptr &src)
{
	imageSrc = src;

	kfid curKf = 0;

	auto imageAnchor = BaseFrame::create(imageSrc->at(0), cameraPars);
	imageAnchor->computeFeatures(bFeats);
	imageDb.addImage(curKf, imageAnchor->allKeypoints(), imageAnchor->allDescriptors());

}

const map<kfid, uint64> getKeyFrameMapping() const
{ return keyframeToBagFrameId; }

const ImageDatabase& getImageDb() const
{ return imageDb; }


protected:
	map<kfid, uint64> keyframeToBagFrameId;
	ImageDatabase imageDb;
	ImageBag::Ptr imageSrc;

	cv::Ptr<cv::DescriptorMatcher> bMatcher;
	cv::Ptr<cv::FeatureDetector> bFeats;

	const CameraPinholeParams cameraPars;
};


int main(int argc, char *argv[])
{
	float frameRate=10.0;

	Vmml::Mapper::ProgramOptions progOptions;
	progOptions.addSimpleOptions("frame-rate", "Reduce image rate", frameRate);
	progOptions.parseCommandLineArgs(argc, argv);

	Vmml::Mapper::RVizConnector rosConn(argc, argv, "index_creator");

	rosbag::Bag &mybag = progOptions.getInputBag();

	/*
	 * Need to reduce frame rate of the bag
	 */
	auto &images = *progOptions.getImageBag();
	vector<uint64> imageRedSamples;
	images.desample(frameRate, imageRedSamples);
	auto tLen = (images.timeAt(imageRedSamples.back())-images.timeAt(imageRedSamples.front())).toSec();
	cout << "Frequency after desampled: " << float(imageRedSamples.size()) / tLen << endl;

	auto camera0 = progOptions.getCameraParameters();

	auto gnssTopic = selectTopicForGnssLocalization(mybag);
	auto trackGnss = TrajectoryGNSS::fromRosBagSatFix(mybag, gnssTopic);
	trackGnss.dump((progOptions.getWorkDir()/"gnss.csv").string());

	Trajectory trackImage;
	kfid curKf = 0;

	ImageDatabase imageDb;

	const int maxLim = images.size();
//	const int maxLim = 1000;

	kfid keyframeId = 0;
	for (int imageBagId: imageRedSamples) {
		auto curImage = BaseFrame::create(images.at(imageBagId), camera0);
		ptime imageTimestamp = images.timeAt(imageBagId).toBoost();
		curImage->computeFeatures(bFeats);

		if (keyframeId==0) {
			// No checks
			imageDb.addImage(keyframeId, curImage->allKeypoints(), curImage->allDescriptors());
		}
		else
			// Perform checks
			imageDb.addImage2(keyframeId, curImage->allKeypoints(), curImage->allDescriptors());

		rosConn.publishPlainBaseFrame(*curImage);
		cout << imageBagId+1 << " / " << maxLim << endl;
		imageDb.keyframeIdToBag[keyframeId] = imageBagId;
		keyframeId += 1;
	}

	// write csv file for mapping from keyframe# -> bagframe#
	writeCsv((progOptions.getWorkDir()/"frames.csv").string(), imageDb.keyframeIdToBag);

	cout << "Done mapping\n";

	imageDb.saveToDisk((progOptions.getWorkDir()/"imagedb-full.dat").string());
	return 0;
}
