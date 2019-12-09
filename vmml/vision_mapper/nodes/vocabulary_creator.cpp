/*
 * vocabulary_creator.cpp
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
#include "utilities.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;


// 5 cm/s
const float
linearVelocityThreshold = 0.05,
linearDistThreshold = 100.0,
linearIdleThreshold = 10.0,
imageSameThrScore = 0.15;


cv::Ptr<cv::DescriptorMatcher> bMatcher = cv::BFMatcher::create();
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		6000,
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

	ImageBag images(mybag, "/camera1/image_raw", 0.5);
	uint width, height;
	images.getImageDimensions(width, height);

	CameraPinholeParams camera0(0, 0, 0, 0, width, height);

	auto trackGnss = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence");
	trackGnss.dump("gnss.csv");

	Mapper::RVizConnector rosConn(argc, argv, "vocabulary_creator");

	Trajectory trackImage;

	auto imageAnchor = BaseFrame::create(images.at(0), camera0);
	imageAnchor->computeFeatures(bFeats);

	for (int i=1; i<images.size(); ++i) {
		auto curImage = BaseFrame::create(images.at(i), camera0);
		ptime imageTimestamp = images.timeAt(i).toBoost();
		curImage->computeFeatures(bFeats);

		float comparisonScore = compareAndScore(*imageAnchor, *curImage);
		bool isKeyFrame=false;

		if (comparisonScore<=imageSameThrScore) {
			imageAnchor = curImage;
			rosConn.publishBaseFrame(*curImage);
			PoseStamped imagePose = trackGnss.at(imageTimestamp);
			trackImage.push_back(imagePose);
			isKeyFrame = true;
		}

		cout << i << " / " << images.size() << (isKeyFrame==true?"*":"") << "; Score: " << comparisonScore << endl;
	}

/*
	Trajectory vIdle;

	// Choose timestamps when the vehicle is idle or has elapsed distance greater than threshold
	bool isIdle = false;
	PoseStamped lastPose, lastIdlePos;
	double elapsedDistance = 0.0;

	for (int i=0; i<track1.size(); ++i) {
		auto poseI = track1[i];
		auto tw = track1.getVelocityAt(i);
		double dist = (lastPose.position()-poseI.position()).norm();
		double distIdle = (poseI.position()-lastIdlePos.position()).norm();

		if (tw.linear.norm()<=linearVelocityThreshold and isIdle==false and distIdle>=linearIdleThreshold) {
			lastIdlePos = poseI;
			vIdle.push_back(poseI);
			isIdle=true;
			elapsedDistance = 0.0;
		}

		else {
			elapsedDistance += dist;
			isIdle=false;
			if (elapsedDistance>=linearDistThreshold) {
				vIdle.push_back(poseI);
				elapsedDistance = 0.0;
			}
		}

		lastPose = poseI;
	}

	for (uint i=0; i<vIdle.size(); ++i) {
		PoseStamped imagePose = vIdle.at(i);
		int imageIdx = images.getPositionAtTime(ros::Time::fromBoost(imagePose.timestamp));
		auto image = images.at(imageIdx);
		cv::imwrite(to_string(imageIdx)+".png", image);
	}

	vIdle.dump("frames_for_vocabulary.txt");
*/

	trackGnss.dump("gnss.csv");
	trackImage.dump("images.csv");
	return 0;
}
