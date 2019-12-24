/*
 * imagelidar_bag_mapper.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: sujiwo
 */


#include <rosbag/bag.h>
#include "vmml/MapBuilderLidar.h"
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

// XXX: Supply your own values!
const TTransform tLidarToCamera = TTransform::from_XYZ_RPY(
	Eigen::Vector3d(0.9, 0.3, -0.6),
	-1.520777, -0.015, -1.5488);

int main(int argc, char *argv[])
{
	// Find vocabulary, mask & velodyne calibration file
	auto maskPath = boost::filesystem::path(ros::package::getPath("vision_core")) / "meidai_mask.png";
	auto calibPath = boost::filesystem::path(ros::package::getPath("vision_mapper")) / "meidai-64e-S2.yaml";
	camera0.mask = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
	camera0 = camera0 * enlarge;

	rosbag::Bag mybag(argv[1]);
	MapBuilderLidar mapBuilderz(camera0);
	mapBuilderz.setTransformationFromLidarToCamera(tLidarToCamera);
	mapBuilderz.run(mybag, calibPath.string(), "/velodyne_packets", "/camera1/image_raw", enlarge);

	// Profit!
	auto visionMapTrajectory = mapBuilderz.getMap()->dumpCameraTrajectory();
	visionMapTrajectory.dump("/tmp/mapping-vision.csv");

	return 0;
}
