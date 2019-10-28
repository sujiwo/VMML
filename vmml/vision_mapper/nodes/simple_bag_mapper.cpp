/*
 * simple_bag_mapper.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include "MapBuilder.h"
#include "RVizConnector.h"
#include "ImageBag.h"
#include "utilities.h"


using namespace std;
using Vmml::MapBuilder;
using Vmml::Mapper::RVizConnector;
using Vmml::ptime;


Vmml::CameraPinholeParams camera0 (
	1150.96938467,
	1150.96938467,
	988.511326762,
	692.803953253,
	1920, 1440);
const float enlarge = 0.333333333333;


int main(int argc, char *argv[])
{
	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", enlarge);

//	RVizConnector rosHdl(argc, argv, "monocular_mapper");

	camera0 = camera0 * enlarge;
	camera0.fps = float(imageBag.size()) / Vmml::toSeconds(imageBag.length().toBoost());

	MapBuilder mapBuilderz(camera0);

	for (int i=0; i<imageBag.size(); ++i) {
		auto imageMsg = imageBag.at(i);
		ptime timestamp = imageBag.timeAt(i).toBoost();
		mapBuilderz.feed(imageMsg, timestamp);
		cout << "Counter: " << i << endl;
	}

	return 0;
}
