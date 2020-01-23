/*
 * testbag.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: sujiwo
 */

#include <rosbag/bag.h>
#include <pcl/io/pcd_io.h>
#include "vmml/TrajectoryGNSS.h"
#include "vmml/utilities.h"
#include "vmml/LidarScanBag.h"
#include "vmml/ImageBag.h"
#include <opencv2/highgui.hpp>

using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag ost(mybag, "/front_rgb/image_raw");
	uint size = ost.size();

	auto img0 = ost.at(14539);
	cv::imwrite("/tmp/0.png", img0);

	return 0;
}
