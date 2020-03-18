/*
 * imagelidar_projection.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: sujiwo
 *
 *  Projection test for Lidar to image
 */

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "vmml/BaseFrame.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;
using namespace Vmml::Mapper;


class LidarImageFrame: public BaseFrame
{
public:

	template <class PointT>
	static cv::Mat drawPointCloud(const BaseFrame &bframe, const pcl::PointCloud<PointT> &cloud)
	{
		cv::Mat frameRes = bframe.getImage().clone();
		// XXX: Unfinished
		return frameRes;
	}

protected:
};


int main(int argc, char *argv[])
{
	ProgramOptions projOptions;
	RVizConnector rosConn(argc, argv, "lidar_projection");
	rosConn.setImageTopicName("lidar_in_image");

	projOptions.parseCommandLineArgs(argc, argv);

	auto lidarBag = projOptions.getLidarScanBag();
	auto imageBag = projOptions.getImageBag();
	auto &imagePipe = projOptions.getImagePipeline();
	auto camera0 = projOptions.getWorkingCameraParameter();

	assert(lidarBag->getBagStartTime() >= imageBag->getBagStartTime());
	assert(lidarBag->getBagStopTime() <= imageBag->getBagStopTime());
	assert(camera0.fx!=0 and camera0.fy!=0);

	for (int fr=0; fr<lidarBag->size(); ++fr) {

		ptime lidarTm;
		double timeDiff;
		auto lidarScan = lidarBag->at<pcl::PointXYZ>(fr, &lidarTm);
		auto imageN = imageBag->getPositionAtTime(ros::Time::fromBoost(lidarTm));
		auto imageTm = imageBag->timeAt(imageN);
		auto imageMt = imageBag->at(imageN);

//		imageMt = imagePipe.run(imageMt);

		auto imageFr = BaseFrame::create(imageMt, camera0);
		cv::Mat frameProj = LidarImageFrame::drawPointCloud(*imageFr, *lidarScan);
		rosConn.publishImage(frameProj, imageTm);
	}

	return 0;
}
