/*
 * imagedb_mapper.cpp
 *
 *  Created on: Nov 18, 2019
 *      Author: sujiwo
 */

#include <sstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <boost/filesystem/convenience.hpp>
#include "vmml/Pose.h"
#include "vmml/ImageBag.h"
#include "vmml/LidarScanBag.h"
#include "vmml/ImageDatabaseBuilder.h"
#include "RVizConnector.h"
#include "ProgramOptions.h"


using namespace std;
using namespace Vmml;


Vmml::Mapper::ProgramOptions progOptions;


int main(int argc, char *argv[])
{
	progOptions.parseCommandLineArgs(argc, argv);

	auto camera0 = progOptions.getCameraParameters();

	ImageDatabaseBuilder::Param idbParams;
	ImageDatabaseBuilder imageDbMapper(idbParams, camera0);
	imageDbMapper.setTranformationFromLidarToCamera(progOptions.getLidarToCameraTransform());

	Vmml::ImageBag imageBag(progOptions.getInputBag(), progOptions.getImageTopic(), progOptions.getImageResizeFactor());
	Vmml::LidarScanBag lidarBag(progOptions.getInputBag(), progOptions.getLidarTopic());

	Mapper::RVizConnector rosConn(argc, argv, "mapper", progOptions.getLidarToCameraTransform());
	rosConn.setMap(imageDbMapper.getMap());

	int limit;
	if (argc>2)
		limit = stoi(argv[2]);
	else limit = lidarBag.size();

	for (int li=0; li<limit; li++) {

		ptime lidarTimestamp, imageTimestamp;
		cv::Mat nearImage;
		auto lidarScan = lidarBag.getUnfiltered<ImageDatabaseBuilder::PointT>(li, &lidarTimestamp);

		try {
			uint imageN = imageBag.getPositionAtTime(ros::Time::fromBoost(lidarTimestamp));
			imageTimestamp = imageBag.timeAt(imageN).toBoost();
			nearImage = imageBag.at(imageN);
		} catch (out_of_range &e) {
			continue;
		}

		auto isKey = imageDbMapper.feed(lidarScan, lidarTimestamp, nearImage, imageTimestamp);
		cout << (isKey ? "*" : "") << li  << " / " << limit << endl;

		rosConn.publishFrameWithLidar(*(imageDbMapper.getLastFrame()));
	}

	auto mapFilenameBasename = boost::filesystem::basename(progOptions.getBagPath());
	auto workDir = progOptions.getWorkDir();
	imageDbMapper.getMap()->save((workDir / (mapFilenameBasename+".vmap")).string());
	imageDbMapper.getTrajectory().dump((workDir / (mapFilenameBasename+"-lidar.csv")).string());
	imageDbMapper.getMap()->dumpCameraTrajectory().dump((workDir/(mapFilenameBasename+"-camera.csv")).string());

	auto mapCloud = imageDbMapper.getMap()->dumpPointCloudFromMapPoints();
	pcl::io::savePCDFileBinary((workDir/(mapFilenameBasename+"-vmap.pcd")).string(), *mapCloud);

	return 0;
}
