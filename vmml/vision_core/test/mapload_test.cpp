/*
 * mapload_test.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: sujiwo
 */

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "VisionMap.h"
#include "BaseFrame.h"
#include "ImageBag.h"

using namespace Vmml;
using namespace std;


int main(int argc, char *argv[])
{
	Vmml::VisionMap::Ptr vMap = Vmml::VisionMap::create();
	vMap->load("/home/sujiwo/VmmlWorkspace/RelWithDebInfo/motoyama.vmap");

	auto cloud = vMap->dumpPointCloudFromMapPoints();
	pcl::io::savePCDFileBinary("/tmp/xmap.pcd", *cloud);

	rosbag::Bag mybag("/Data/MapServer/Logs/log_2016-12-26-13-21-10.bag");
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", 0.3333333);

	auto mtFrame = BaseFrame::create(imageBag.at(54499), vMap->getCameraParameter(0));
	auto candidates = vMap->findCandidates(*mtFrame);
	cout << "Found " << candidates.size() << endl;
	for (auto kf: candidates) {
		cout << kf << ' ';
	}
	return 0;
}
