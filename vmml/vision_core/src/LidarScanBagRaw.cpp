/*
 * LidarScanBag.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */



#include "vmml/LidarScanBagRaw.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>


using namespace std;
using velodyne_rawdata::VPoint;
using velodyne_rawdata::VPointCloud;
using pcl::PointCloud;


namespace Vmml {

LidarScanBagRaw::LidarScanBagRaw(
	rosbag::Bag const &bag,
	const std::string &topic,
	const ros::Time &startTime,
	const ros::Time &endTime,
	const std::string &velodyneCalibrationFile,
	float _velodyneMinRange,
	float _velodyneMaxRange) :

		RandomAccessBag(bag, topic, startTime, endTime),
		data_(new velodyne_rawdata::RawData())
{
	prepare(velodyneCalibrationFile, _velodyneMinRange, _velodyneMaxRange);
}


LidarScanBagRaw::LidarScanBagRaw(
	rosbag::Bag const &bag,
	const std::string &topic,
	const double seconds1FromOffset,
	const double seconds2FromOffset,
	const std::string &velodyneCalibrationFile,
	float _velodyneMinRange,
	float _velodyneMaxRange) :

		RandomAccessBag(bag, topic, seconds1FromOffset, seconds2FromOffset),
		data_(new velodyne_rawdata::RawData())
{
	prepare(velodyneCalibrationFile, _velodyneMinRange, _velodyneMaxRange);
}



void
LidarScanBagRaw::prepare(const string &lidarCalibFile,
		float _velodyneMinRange,
		float _velodyneMaxRange)
{
	string fname;
	if (lidarCalibFile.empty()) {
		boost::filesystem::path vlp (ros::package::getPath("velodyne_pointcloud"));
		vlp /= "params/64e_s2.1-sztaki.yaml";
		fname = vlp.string();
		cerr << "Bug: using default Velodyne Calibration Parameter" << endl;
	}
	else fname = lidarCalibFile;

	if (data_->setupOffline(fname, velodyneMaxRange, velodyneMinRange)
		== -1)
		throw runtime_error("Unable to set velodyne converter");

	data_->setParameters(_velodyneMinRange, _velodyneMaxRange, velodyneViewDirection, velodyneViewWidth);
}


template<>
void mPointCloud<pcl::PointXYZ>::addPoint(const float& x, const float& y, const float& z,
			const uint16_t& ring,
			const uint16_t& azimuth,
			const float& distance,
			const float& intensity)
{
	// convert polar coordinates to Euclidean XYZ
	pcl::PointXYZ point;
	point.x = x;
	point.y = y;
	point.z = z;

	// append this point to the cloud
	pc->points.push_back(point);
	++pc->width;
}


template<>
void
mPointCloud<pcl::PointXYZI>::addPoint(
	const float& x, const float& y, const float& z,
	const uint16_t& ring,
	const uint16_t& azimuth,
	const float& distance,
	const float& intensity)
{
	// convert polar coordinates to Euclidean XYZ
	pcl::PointXYZI point;
	point.x = x;
	point.y = y;
	point.z = z;
	point.intensity = intensity;

	// append this point to the cloud
	pc->points.push_back(point);
	++pc->width;
}

}		/* namespace Vmml */
