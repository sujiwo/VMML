/*
 * MapBuilderLidar.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include "MapBuilderLidar.h"


using namespace std;


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
	const std::string &imageTopic)
{

}


} /* namespace Vmml */
