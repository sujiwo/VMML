/*
 * test_vo.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include "VisualOdometry.h"
#include "utilities.h"
#include "ImageBag.h"


using namespace Vmml;
using namespace std;


Vmml::CameraPinholeParams camera0 (
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920, 1440);	// width, height
const float enlarge = 0.333333333333;


int main(int argc, char *argv[])
{
	VisualOdometry::Parameters voPars;
	camera0.mask = cv::imread((boost::filesystem::path(ros::package::getPath("vision_mapper")) / "meidai_mask.png").string(), cv::IMREAD_GRAYSCALE);
	voPars.camera = camera0 * enlarge;

	VisualOdometry VoRunner(voPars);

	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", enlarge);

	int limit;
	if (argc>=2)
		limit = stoi(argv[2]);
	else limit = imageBag.size();

	for (int n=0; n<limit; ++n) {
		auto imageMsg = imageBag.at(n);
		ptime timestamp = imageBag.timeAt(n).toBoost();
		VoRunner.process(imageMsg, timestamp);
		cout << n << ": " << VoRunner.getInlier() << endl;
	}
	cout << "Done\n";

	const auto voTrack = VoRunner.getTrajectory();
	const auto cloudBuild = VoRunner.getPoints();
	voTrack.dump("/tmp/x.csv");
	pcl::io::savePCDFileBinary("/tmp/mapVoTest.pcd", *cloudBuild);

	return 0;
}
