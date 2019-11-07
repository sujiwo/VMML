/*
 * MapBuilderLidar.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include "MapBuilderLidar.h"


using namespace std;
using pcl::PointXYZ;


namespace Vmml {


MapBuilderLidar::MapBuilderLidar(const CameraPinholeParams &camera0, const std::string &mapVocabularyPath) :
	MapBuilder(camera0, mapVocabularyPath)
{
	// TODO Auto-generated constructor stub

}


MapBuilderLidar::~MapBuilderLidar()
{
	// TODO Auto-generated destructor stub
}


void
MapBuilderLidar::run(
	const rosbag::Bag &bagFd,
	const std::string &velodyneCalibrationFilePath,
	const std::string &velodyneTopic,
	const std::string &imageTopic,
	const float imageScale)
{
	imageSource.reset(new ImageBag(bagFd, imageTopic, imageScale));
	velScanSource.reset(new LidarScanBag(bagFd, velodyneTopic, velodyneCalibrationFilePath));

	const int N = velScanSource->size();
	for (int ild=0; ild<N; ++ild) {

		// Fetch lidar scan
		ptime lidarTs;
		auto lidarScan = velScanSource->getUnfiltered<PointXYZ>(0, &lidarTs);

		// initialized ?
	}
}


} /* namespace Vmml */
