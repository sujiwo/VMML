/*
 * MapBuilderLidar.h
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_MAPBUILDERLIDAR_H_
#define VMML_MAPPER_MAPBUILDERLIDAR_H_

#include <string>
#include "utilities.h"
#include "RandomAccessBag.h"
#include "MapBuilder.h"
#include "LidarScanBag.h"
#include "ImageBag.h"


namespace Vmml {

class MapBuilderLidar : public Vmml::MapBuilder
{
public:
	MapBuilderLidar(const CameraPinholeParams &camera0, const std::string &mapVocabularyPath="");
	virtual ~MapBuilderLidar();

	void run(
		const rosbag::Bag &bagFd,
		const std::string &velodyneCalibrationFilePath,
		const std::string &velodyneTopic,
		const std::string &imageTopic,
		const float imageScale=1.0);

protected:

	ImageBag::Ptr imageSource;
	LidarScanBag::Ptr velScanSource;
};

} /* namespace Vmml */

#endif /* VMML_MAPPER_MAPBUILDERLIDAR_H_ */
