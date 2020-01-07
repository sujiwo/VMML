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
	Vmml::Mapper::ProgramOptions progOptions;
	progOptions.parseCommandLineArgs(argc, argv);

	Vmml::Mapper::RVizConnector rosConn(argc, argv, "index_creator");

	rosbag::Bag &mybag = progOptions.getInputBag();

	auto &images = *progOptions.getImageBag();

	auto camera0 = progOptions.getCameraParameters();

	auto gnssTopic = selectTopicForGnssLocalization(mybag);
	auto trackGnss = TrajectoryGNSS::fromRosBagSatFix(mybag, gnssTopic);
	trackGnss.dump((progOptions.getWorkDir()/"gnss.csv").string());

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
			string imgName = (progOptions.getWorkDir() / ("kf" + to_string(curKf) + ".png")).string();
			cv::imwrite(imgName, curImage->getImage());
			rosConn.publishPlainBaseFrame(*curImage);
		}

		cout << i+1 << " / " << maxLim << (isKeyFrame==true?"*":"") << endl;
	}

	trackImage.dump((progOptions.getWorkDir() / "images.csv").string());

	// write csv file for mapping from keyframe# -> bagframe#
	writeCsv((progOptions.getWorkDir()/"frames.csv").string(), kfToFrameNum);

	cout << "Done mapping\n";

	imageDb.saveToDisk((progOptions.getWorkDir()/"imagedb-2000.dat").string());
	return 0;
}
