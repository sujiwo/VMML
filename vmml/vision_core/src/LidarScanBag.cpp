/*
 * LidarScanBag.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */



#include <exception>
#include "vmml/LidarScanBag.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>


using namespace std;
using pcl::PointCloud;


namespace Vmml {

LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag,
	const std::string &topic) :

		RandomAccessBag(bag, topic)
{
	if (messageType() != "sensor_msgs/PointCloud2")
		throw invalid_argument("Requested topic is not of type sensor_msgs/PointCloud2");
}


using pcl::PointXYZI;
template<>
void LidarScanBag::doConvertTemporary<PointXYZI>(const sensor_msgs::PointCloud2 &src, pcl::PointCloud<PointXYZI> &dst)
{
	dst.reserve(src.data.size());
	uint i = 0;
	for (auto &p: src) {
		PointXYZI pd;
		pd.x = p.x; pd.y = p.y; pd.z = p.z; pd.intensity = p.intensity;
		dst[i] = pd;
		i++;
	}
}


using pcl::PointXYZ;
template<>
void LidarScanBag::doConvertTemporary<pcl::PointXYZ>(const sensor_msgs::PointCloud2 &src, pcl::PointCloud<pcl::PointXYZ> &dst)
{
	dst.resize(src.size());
	uint i = 0;
	for (auto &p: src) {
		PointXYZ pd;
		pd.x = p.x; pd.y = p.y; pd.z = p.z;
		dst[i] = pd;
		i++;
	}
}



/*
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
*/

}		/* namespace Vmml */
