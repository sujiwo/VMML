/*
 * testbag.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: sujiwo
 */

#include <rosbag/bag.h>
#include "TrajectoryGNSS.h"
#include "utilities.h"

using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	auto track1 = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence").setToOrigin();
	auto track2 = TrajectoryGNSS::fromRosBag2(mybag, "/nmea_sentence").setToOrigin();

	track1.dump("test-method1.csv");
	track2.dump("test-method2.csv");

	return 0;
}
