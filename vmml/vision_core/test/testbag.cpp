/*
 * testbag.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: sujiwo
 */

#include <string>
#include <rosbag/bag.h>
#include <pcl/io/pcd_io.h>
#include "vmml/TrajectoryGNSS.h"
#include "vmml/utilities.h"
#include "vmml/LidarScanBag.h"
#include "vmml/ImageBag.h"
#include "vmml/ImagePreprocessor.h"
#include <opencv2/highgui.hpp>

using namespace std;
using namespace Vmml;

float alpha = 0.3975;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag ost(mybag, "/front_rgb/image_raw");

	auto img0 = ost.at(stoi(argv[2]), true);
	auto imgp = ImagePreprocessor::toIlluminatiInvariant(img0, alpha);
	cv::imwrite("/tmp/raw.png", img0);
	cv::imwrite("/tmp/proc.png", imgp);

	return 0;
}
