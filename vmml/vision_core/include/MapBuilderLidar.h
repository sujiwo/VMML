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
#include "LocalLidarMapper.h"


namespace Vmml {

class MapBuilderLidar : public Vmml::MapBuilder
{
public:
	MapBuilderLidar(const CameraPinholeParams &camera0, const std::string &mapVocabularyPath="");
	virtual ~MapBuilderLidar();

	struct LidarImageFrame : public MapBuilder::TmpFrame
	{
		typedef std::shared_ptr<LidarImageFrame> Ptr;

		LidarImageFrame(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &scan, std::shared_ptr<VisionMap> &_parent, ptime lidarTs);

		static Ptr create(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &scan, std::shared_ptr<VisionMap> &_parent, ptime lidarTs);

		inline void setImage(cv::Mat i)
		{ image = i; }

		LocalLidarMapper::CloudType::ConstPtr lidarScan;
		ptime lidarTs;
	};

	void run(
		const rosbag::Bag &bagFd,
		const std::string &velodyneCalibrationFilePath,
		const std::string &velodyneTopic,
		const std::string &imageTopic,
		const float imageScale=1.0);

	cv::Mat getImage(const ptime &ts, ptime &imageTs);

protected:

	ImageBag::Ptr imageSource;
	LidarScanBag::Ptr velScanSource;

	LidarImageFrame::Ptr currentFrame = nullptr;

	bool track();

	LocalLidarMapper lidarTracker;
};

} /* namespace Vmml */

#endif /* VMML_MAPPER_MAPBUILDERLIDAR_H_ */