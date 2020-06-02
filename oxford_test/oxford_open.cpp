/*
 * oxford_open.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "OxfordDataset.h"

using namespace std;
using namespace oxf;


int main(int argc, char *argv[])
{
	OxfordDataset dataset(argv[1]);

	cout << dataset.hz() << " Hz\n";
	auto resamples = dataset.desample(7.0);

	rosbag::Bag oxBag("/Data/test.bag", rosbag::BagMode::Write);
	int i=0;
	for (auto ix: resamples) {
		auto imgp = dataset.at(ix);

		cv_bridge::CvImage cvImg;
		cvImg.encoding = sensor_msgs::image_encodings::BGR8;
		cvImg.image = imgp.center_image;
		auto ts = ros::Time::fromBoost(imgp.timestamp);
		cvImg.header.stamp = ts;

		oxBag.write("oxford", ts, cvImg.toImageMsg());
		cout << i++ << '/' << resamples.size() << endl;
	}

	oxBag.close();
	return 0;
}
