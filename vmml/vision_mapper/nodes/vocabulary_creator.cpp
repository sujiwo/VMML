/*
 * vocabulary_creator.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: sujiwo
 */

#include <vector>
#include <rosbag/bag.h>
#include "VisionMap.h"
#include "Trajectory.h"
#include "TrajectoryGNSS.h"
#include "utilities.h"

using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	auto track1 = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence").setToOrigin();
	track1.dump("gnss.csv");

	return 0;
}
