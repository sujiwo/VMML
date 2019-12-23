/*
 * testbag.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: sujiwo
 */

#include <rosbag/bag.h>
#include <pcl/io/pcd_io.h>
#include "vmml/TrajectoryGNSS.h"
#include "vmml/utilities.h"
#include "vmml/LidarScanBag.h"

using namespace std;
using namespace Vmml;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

/*
	auto track1 = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence").setToOrigin();
	auto track2 = TrajectoryGNSS::fromRosBag2(mybag, "/nmea_sentence").setToOrigin();

	track1.dump("test-method1.csv");
	track2.dump("test-method2.csv");
*/

	LidarScanBag ost(mybag, "/points_raw");
	auto mcloud = ost.at<pcl::PointXYZ>(100);
	pcl::io::savePCDFileBinary("/tmp/100.pcd", *mcloud);

	return 0;
}
