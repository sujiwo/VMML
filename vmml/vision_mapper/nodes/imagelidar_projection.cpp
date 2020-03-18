/*
 * imagelidar_projection.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: sujiwo
 */

#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace Vmml;
using namespace Vmml::Mapper;


int main(int argc, char *argv[])
{
	ProgramOptions projOptions;
	RVizConnector rosConn(argc, argv, "lidar_projection");

	projOptions.parseCommandLineArgs(argc, argv);

	auto lidarBag = projOptions.getLidarScanBag();
	auto imageBag = projOptions.getImageBag();
	auto &imagePipe = projOptions.getImagePipeline();

	return 0;
}
