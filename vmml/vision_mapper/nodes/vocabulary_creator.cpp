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

using namespace std;
using namespace Vmml;


// 5 cm/s
const float
linearVelocityThreshold = 0.05,
linearDistThreshold = 100.0,
linearIdleThreshold = 10.0;


float compareAndScore(const BaseFrame &f1, const BaseFrame &f2)
{
	Matcher::PairList featurePairs;
	int N1 = Matcher::matchBruteForce(f1, f2, featurePairs);
	if (N1==0) return 0;

}


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag images(mybag, "/camera1/image_raw", 0.6666666666667);

	auto track1 = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence");
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

	return 0;
}
