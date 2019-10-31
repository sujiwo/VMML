/*
 * simple_bag_mapper.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <opencv2/highgui.hpp>
#include "MapBuilder.h"
#include "RVizConnector.h"
#include "ImageBag.h"
#include "utilities.h"


using namespace std;
using namespace std::placeholders;
using Vmml::MapBuilder;
using Vmml::Mapper::RVizConnector;
using Vmml::ptime;


Vmml::CameraPinholeParams camera0 (
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920, 1440);	// width, height
const float enlarge = 0.333333333333;


int main(int argc, char *argv[])
{
	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", enlarge);

	auto maskPath = Vmml::getMyPath() / "samples/meidai_mask.png";
	camera0.mask = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
	camera0 = camera0 * enlarge;
	camera0.fps = float(imageBag.size()) / Vmml::toSeconds(imageBag.length().toBoost());

	MapBuilder mapBuilderz(camera0);

	RVizConnector rosHdl(argc, argv, "monocular_mapper");
	auto fx=std::bind<void>(&RVizConnector::publishKeyFrame, &rosHdl, std::placeholders::_1);
	mapBuilderz.registerKeyFrameCallback(fx);

	// XXX: Put interrupt (ctrl-c) signal handler prior to entering loop

	auto N = imageBag.size();
//	auto N = 233;
	for (int i=0; i<N; ++i) {
		auto imageMsg = imageBag.at(i);
		ptime timestamp = imageBag.timeAt(i).toBoost();
		mapBuilderz.feed(imageMsg, timestamp);
		cout << "Counter: " << i << endl;
	}

	return 0;
}
