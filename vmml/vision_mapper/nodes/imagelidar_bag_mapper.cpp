/*
 * imagelidar_bag_mapper.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: sujiwo
 */


#include <rosbag/bag.h>
#include "MapBuilderLidar.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;


CameraPinholeParams camera0 (
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920, 1440);	// width, height
const float enlarge = 0.333333333333;


int main(int argc, char *argv[])
{
	// Find vocabulary, mask & velodyne calibration file
	auto vocabPath = boost::filesystem::path(ros::package::getPath("vision_core")) / "ORBvoc.txt";
	auto maskPath = boost::filesystem::path(ros::package::getPath("vision_core")) / "meidai_mask.png";
	auto calibPath = boost::filesystem::path(ros::package::getPath("vision_mapper")) / "meidai-64e-S2.yaml";
	camera0.mask = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
	camera0 = camera0 * enlarge;

	rosbag::Bag mybag(argv[1]);
	MapBuilderLidar mapBuilderz(camera0, vocabPath.string());
	mapBuilderz.run(mybag, calibPath.string(), "/velodyne_packets", "/camera1/image_raw", enlarge);

	return 0;
}