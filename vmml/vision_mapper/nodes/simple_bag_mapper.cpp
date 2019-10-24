/*
 * simple_bag_mapper.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */


#include "MapBuilder.h"
#include "RVizConnector.h"
#include "ImageBag.h"


using namespace std;
using Vmml::MapBuilder;
using Vmml::Mapper::RVizConnector;


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

	camera0 = camera0 * enlarge;
	MapBuilder mapBuilderz(camera0);

	for (int i=180; i<imageBag.size(); ++i) {
		auto imageMsg = imageBag.at(i);
		mapBuilderz.feed(imageMsg);
	}

	return 0;
}
