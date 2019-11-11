/*
 * test_vo.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: sujiwo
 */


#include <opencv2/highgui.hpp>
#include "VisualOdometry.h"
#include "utilities.h"
#include "ImageBag.h"


using namespace Vmml;


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
	voPars.camera = camera0 * enlarge;
	voPars.camera.mask = cv::imread((boost::filesystem::path(ros::package::getPath("vision_mapper")) / "meidai_mask.png").string(), cv::IMREAD_GRAYSCALE);

	VisualOdometry VoRunner(voPars);

	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", enlarge);

	for (int n=0; n<imageBag.size(); ++n) {
		auto imageMsg = imageBag.at(n);
		ptime timestamp = imageBag.timeAt(n).toBoost();
	}

	return 0;
}
