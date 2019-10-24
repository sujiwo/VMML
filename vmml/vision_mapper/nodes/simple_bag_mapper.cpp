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


int main(int argc, char *argv[])
{
	rosbag::Bag mybag(argv[1]);
	Vmml::ImageBag imageBag(mybag, "/camera1/image_raw", 0.333333333);

	return 0;
}
