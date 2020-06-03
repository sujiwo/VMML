/*
 * tobag.cpp
 *
 *  Created on: Jun 3, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <exception>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include "OxfordDataset.h"

using namespace std;


int main(int argc, char *argv[])
{
	oxf::OxfordDataset dataset(argv[1]);

	rosbag::Bag targetBag(argv[2], rosbag::BagMode::Write);

	for (int i=0; i<dataset.size(); ++i) {
		auto record = dataset.at(i);
		auto msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", record.center_image).toCompressedImageMsg(cv_bridge::Format::PNG);
		msg->header.frame_id = "center";
		msg->header.seq = i;
		msg->header.stamp = ros::Time::fromBoost(record.timestamp);
		targetBag.write("center", msg->header.stamp, msg);
		cout << i+1 << '/' << dataset.size() << endl;
	}

	targetBag.close();
	return 0;
}
