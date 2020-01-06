/*
 * LidarScanBag2.h
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#ifndef _LIDARSCANBAGRAW_H_
#define _LIDARSCANBAGRAW_H_


#include <string>
#include <exception>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include "LidarScanBag.h"

#include "utilities.h"
#include "RandomAccessBag.h"


namespace Vmml {


template<typename PointT>
using ScanPtr = boost::shared_ptr<pcl::PointCloud<PointT>>;

template<typename PointT>
using ScanConstPtr = boost::shared_ptr<const pcl::PointCloud<PointT>>;

/*
 * XXX: These values may need to be adjusted
 */
const float
	velodyneMinRange = 5.0,
	velodyneMaxRange = 130,
	velodyneViewDirection = 0,
	velodyneViewWidth = 2*M_PI;


template<typename PointT>
class mPointCloud : public velodyne_rawdata::DataContainerBase
{
public:
	ScanPtr<PointT> pc;

	mPointCloud() : pc(new pcl::PointCloud<PointT>) {}

	void addPoint(const float& x, const float& y, const float& z,
		const uint16_t& ring,
		const uint16_t& azimuth,
		const float& distance,
		const float& intensity);
};


template<> void mPointCloud<pcl::PointXYZ>::addPoint(
	const float& x, const float& y, const float& z,
	const uint16_t& ring,
	const uint16_t& azimuth,
	const float& distance,
	const float& intensity);

template<> void mPointCloud<pcl::PointXYZI>::addPoint(
	const float& x, const float& y, const float& z,
	const uint16_t& ring,
	const uint16_t& azimuth,
	const float& distance,
	const float& intensity);


/*
 * Representation of Rosbag Velodyne Scan as point cloud
 * Contrarily from parent class, this class reads velodyne pointcloud directly
 * as velodyne_msgs/VelodyneScan. It requires calibration file.
 */
class LidarScanBagRaw : public LidarScanBag
{
public:

	typedef std::shared_ptr<LidarScanBagRaw> Ptr;

	LidarScanBagRaw(rosbag::Bag const &bag, const std::string &topic,
		const std::string &velodyneCalibrationFile,
		const float _velodyneMinRange=velodyneMinRange, const float _velodyneMaxRange=velodyneMaxRange);


protected:

	boost::shared_ptr<velodyne_rawdata::RawData> data_;

	void prepare(const std::string &lidarCalibFile,
		float _velodyneMinRange,
		float _velodyneMaxRange);

	template<typename PointT>
	ScanConstPtr<PointT>
	convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg)
	{
		mPointCloud<PointT> outPoints;
		outPoints.pc->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
		outPoints.pc->header.frame_id = bagmsg->header.frame_id;
		outPoints.pc->height = 1;

		outPoints.pc->points.reserve(bagmsg->packets.size() * data_->scansPerPacket());

		for (int i=0; i<bagmsg->packets.size(); ++i) {
			data_->unpack(bagmsg->packets[i], outPoints);
		}

		if (filtered) {
			return VoxelGridFilter<PointT>(outPoints.pc);
		}
		else
			return outPoints.pc;
	}

	inline virtual const std::string requiredMessageType()
	{ return "velodyne_msgs/VelodyneScan"; }

};

}		/* namespace Vmml */

#endif /* _LIDARSCANBAGRAW_H_ */
