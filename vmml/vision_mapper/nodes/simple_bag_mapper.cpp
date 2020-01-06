/*
 * simple_bag_mapper.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/highgui.hpp>
#include "vmml/MapBuilder.h"
#include "RVizConnector.h"
#include "vmml/ImageBag.h"
#include "vmml/utilities.h"
#include "ProgramOptions.h"


using namespace std;
using namespace std::placeholders;
using Vmml::MapBuilder;
using Vmml::Mapper::RVizConnector;
using Vmml::ptime;
using Vmml::Pose;


Vmml::CameraPinholeParams camera0 (
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920, 1440);	// width, height
const float enlarge = 0.333333333333;

RVizConnector *rosHdl;


void textfileFrameHandler(const Vmml::MapBuilder::TmpFrame &frame)
{
	static ofstream logFileFd;

	if (logFileFd.is_open()==false) {
		cerr << "Log file is used" << endl;
		logFileFd.open("simple_bag_mapper.csv", ios::trunc);
	}

	Pose pframe = frame.pose();
	logFileFd << pframe.x() << " " << pframe.y() << " " << pframe.z() << endl;
}


int main(int argc, char *argv[])
{
	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", enlarge);

	// Find mask
	auto maskPath = boost::filesystem::path(ros::package::getPath("vision_mapper")) / "meidai_mask.png";
	camera0.mask = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
	camera0 = camera0 * enlarge;
	camera0.fps = float(imageBag.size()) / Vmml::toSeconds(imageBag.length().toBoost());

	MapBuilder mapBuilderz(camera0);

	/*
	 * Disable ROS connector when debugging (set __NOROS environment variable to 1)
	 */
	auto checkDebug=getenv("__NOROS");
	if (checkDebug==NULL or strcmp(checkDebug, "1")!=0) {
		cerr << "ROS is used" << endl;
		rosHdl = new RVizConnector(argc, argv, "monocular_mapper");
		auto fx=std::bind<void>(&RVizConnector::publishFrame, rosHdl, std::placeholders::_1);
		mapBuilderz.registerFrameCallback(fx);
	}
	else {
		mapBuilderz.registerFrameCallback(textfileFrameHandler);
	}

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
