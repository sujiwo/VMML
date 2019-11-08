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


MapBuilderLidar::LidarImageFrame::LidarImageFrame(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &_scan, std::shared_ptr<VisionMap> &_parent, ptime _lidarTs) :
	TmpFrame(img, _parent),
	lidarScan(_scan),
	lidarTs(_lidarTs)
{}


MapBuilderLidar::LidarImageFrame::Ptr
MapBuilderLidar::LidarImageFrame::create(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &_scan, std::shared_ptr<VisionMap> &_parent, ptime _lidarTs)
{ return Ptr(new LidarImageFrame(img, _scan, _parent, _lidarTs)); }


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
		// Delay fetching images
		currentFrame = LidarImageFrame::create(cv::Mat(), lidarScan, vMap, lidarTs);

		// Run NDT
		LocalLidarMapper::ScanProcessLog cLog;
		lidarTracker.feed(lidarScan, lidarTs, cLog);

		// first frame ?
		if (lastAnchor==0) {
			ptime imageTs;
			currentFrame->setImage(getImage(lidarTs, imageTs));
			auto K1 = KeyFrame::fromBaseFrame(*currentFrame, vMap, 0, imageTs);
			vMap->addKeyFrame(K1);
			lastAnchor = K1->getId();
			callFrameFunction();
		}

		else {
			if (cLog.hasScanFrame==true) {
				ptime imageTs;
				currentFrame->setImage(getImage(lidarTs, imageTs));
				track();
			}
		}

	}
}


cv::Mat
MapBuilderLidar::getImage(const ptime &ts, ptime &imageTs)
{
	uint n = imageSource->getPositionAtTime(ros::Time::fromBoost(ts));
	imageTs = imageSource->timeAt(n).toBoost();
	return imageSource->at(n);
}


/*
 * This function performs image-side of tracking
 */
bool
MapBuilderLidar::track()
{
	if (hasInitialized==false) {

	}

	else {

	}

	return true;
}


} /* namespace Vmml */
