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
#include "ImagePipeline.h"
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
numberOfFeatures = 6000;


cv::Ptr<cv::DescriptorMatcher> bMatcher = cv::BFMatcher::create();
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		numberOfFeatures,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
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
			10,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			10,
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


PoseStamped interExtrapolate(const Trajectory &track, const ptime &t)
{
	if (t<track.front().timestamp or t>track.back().timestamp)
		return track.extrapolate(t);
	else
		return track.interpolate(t);
}


int main(int argc, char *argv[])
{
	float frameRate=10.0;
	string mapFilename = "imagedb-full.dat";
	float startTimeSeconds=0;
	float maxSecondsFromStart=-1;

	Vmml::Mapper::ProgramOptions progOptions;
	progOptions.addSimpleOptions("frame-rate", "Reduce image rate", &frameRate);
	progOptions.addSimpleOptions("map-name", "Resulting map file name in working directory", &mapFilename);
	progOptions.addSimpleOptions("start-time", "Mapping will start from x seconds", &startTimeSeconds);
	progOptions.addSimpleOptions("stop-time", "Maximum seconds from start", &maxSecondsFromStart);

	progOptions.parseCommandLineArgs(argc, argv);

	Vmml::Mapper::RVizConnector rosConn(argc, argv, "index_creator");

	rosbag::Bag &mybag = progOptions.getInputBag();

	auto &images = *progOptions.getImageBag();

	/*
	 * Need to reduce frame rate of the bag
	 */
	images.setTimeConstraint(startTimeSeconds, maxSecondsFromStart);

	const int maxLim = images.size();

	RandomAccessBag::DesampledMessageList imageRedSamples;
	images.desample(frameRate, imageRedSamples);
	auto tLen = (images.timeAt(imageRedSamples.back())-images.timeAt(imageRedSamples.front())).toSec();
	cout << "Frequency after desampled: " << float(imageRedSamples.size()) / tLen << endl;
	cout << "# of target frames: " << imageRedSamples.size() << endl;

	auto camera0 = progOptions.getWorkingCameraParameter();

	try {
		auto gnssTopic = selectTopicForGnssLocalization(mybag);
		auto trackGnss = TrajectoryGNSS::fromRosBagSatFix(mybag, gnssTopic);
		trackGnss.transform(progOptions.getGnssOffset());
		trackGnss.dump((progOptions.getWorkDir()/"gnss.csv").string());
		cout << "GNSS dumped to " << (progOptions.getWorkDir()/"gnss.csv").string() << endl;

	} catch (exception &e) {
		cout << "GNSS unavailable\n";
	}

	auto imgPipe = progOptions.getImagePipeline();

	Trajectory trackImage;
	kfid curKf = 0;

	ImageDatabase imageDb;

	kfid keyframeId = 0;
	for (int imageBagId: imageRedSamples) {
		cout << imageBagId+1 << " / " << maxLim;

		auto curImage = images.at(imageBagId);
		cv::Mat mask, preImage;
		imgPipe.run(curImage, preImage, mask);

		auto curFrame = BaseFrame::create(preImage, camera0);
		ptime imageTimestamp = images.timeAt(imageBagId).toBoost();
		curFrame->computeFeatures(bFeats, mask);

		auto imagePosByGnss = (imageTimestamp<trackGnss.front().timestamp or imageTimestamp>trackGnss.back().timestamp) ?
			trackGnss.extrapolate(imageTimestamp) :
			trackGnss.interpolate(imageTimestamp);

		auto t1 = getCurrentTime();
		if (keyframeId==0) {
			// No checks
			imageDb.addImage(keyframeId, curFrame->allKeypoints(), curFrame->allDescriptors());
		}
		else
			// Perform checks
			imageDb.addImage2(keyframeId, curFrame->allKeypoints(), curFrame->allDescriptors());
		auto t2 = getCurrentTime();

		rosConn.publishKeyPointsInFrame(*curFrame);
		trackImage.push_back(imagePosByGnss);
		rosConn.publishTrajectory(trackImage, ros::Time::fromBoost(imageTimestamp));

		cout << ", " << toSeconds(t2-t1) << endl;
		imageDb.keyframeIdToBag[keyframeId] = images.getOriginalZeroIndex()+imageBagId;
		keyframeId += 1;
	}

	// write csv file for mapping from keyframe# -> bagframe#
	writeCsv((progOptions.getWorkDir()/"frames.csv").string(), imageDb.keyframeIdToBag);

	cout << "Done mapping\n";

	auto completeMapName = progOptions.getWorkDir() / mapFilename;
	imageDb.saveToDisk(completeMapName.string());
	cout << "Saved to " << completeMapName.string() << endl;
	return 0;
}
