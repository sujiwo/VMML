/*
 * bag_trajectory.cpp
 *
 *  Created on: Jan 14, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <vmml/TrajectoryGNSS.h>
#include "RandomAccessBag.h"

using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	rosbag::Bag inputBag(argv[1]);
	auto topics = RandomAccessBag::getTopicList(inputBag);

	string msgType, gnssTopic, imageTopic;
	for (auto &t: topics) {
		if (t.second=="sensor_msgs/Image") {
			imageTopic = t.first;
			continue;
		}

		if (t.second=="sensor_msgs/NavSatFix" or t.second=="nmea_msgs/Sentence") {
			msgType=t.second;
			gnssTopic=t.first;
		}
	}
	if (msgType.empty()) {
		cerr << "GNSS topics not found\n";
		exit(1);
	}

	Trajectory gnssTrack, imageTrack;

	if (msgType=="sensor_msgs/NavSatFix") {
		gnssTrack = TrajectoryGNSS::fromRosBagSatFix(inputBag, gnssTopic);
	}

	else if(msgType=="nmea_msgs/Sentence") {
		gnssTrack = TrajectoryGNSS::fromRosBag(inputBag, gnssTopic);
	}

	RandomAccessBag imageInp (inputBag, imageTopic);
	for (int i=0; i<imageInp.size(); ++i) {
		auto imageTimestamp = imageInp.timeAt(i).toBoost();
		PoseStamped intrImagePose;
		if (imageTimestamp < gnssTrack.front().timestamp or imageTimestamp > gnssTrack.back().timestamp)
			intrImagePose = gnssTrack.extrapolate(imageTimestamp);
		else
			intrImagePose = gnssTrack.interpolate(imageTimestamp);
		imageTrack.push_back(PoseStamped(intrImagePose, imageTimestamp));
	}

	imageTrack.dump();

	return 0;
}
