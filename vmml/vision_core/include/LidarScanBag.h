/*
 * LidarScanBag2.h
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#ifndef _LIDARSCANBAG2_H_
#define _LIDARSCANBAG2_H_


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

//#include "utilities.h"
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
	velodyneMinRange = 2.0,
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
 */
class LidarScanBag : public RandomAccessBag
{
public:

	typedef std::shared_ptr<LidarScanBag> Ptr;

	LidarScanBag(
		rosbag::Bag const &bag,
		const std::string &topic,
		const ros::Time &startTime = ros::TIME_MIN,
		const ros::Time &endTime = ros::TIME_MAX,
		const std::string &velodyneCalibrationFile=std::string(),
		float _velodyneMinRange = velodyneMinRange,
		float _velodyneMaxRange = velodyneMaxRange);

	LidarScanBag(
		rosbag::Bag const &bag,
		const std::string &topic,
		const double seconds1FromOffset,
		const double seconds2FromOffset,
		const std::string &velodyneCalibrationFile=std::string(),
		float _velodyneMinRange = velodyneMinRange,
		float _velodyneMaxRange = velodyneMaxRange);


	template<typename PointT>
	ScanConstPtr<PointT>
	at(int position, boost::posix_time::ptime *msgTime=nullptr)
	{
		auto msgP = RandomAccessBag::at<velodyne_msgs::VelodyneScan>(position);
		if (msgTime!=nullptr)
			*msgTime = msgP->header.stamp.toBoost();
		return convertMessage<PointT>(msgP);
	}

	template<typename PointT>
	ScanConstPtr<PointT>
	getFiltered(int position, boost::posix_time::ptime *msgTime=nullptr)
	{
		auto filterState = filtered;
		filtered = true;
		auto scan = at<PointT>(position, msgTime);
		filtered = filterState;
		return scan;
	}

	template<typename PointT>
	ScanConstPtr<PointT>
	getUnfiltered(int position, boost::posix_time::ptime *msgTime=nullptr)
	{
		auto filterState = filtered;
		filtered = false;
		auto scan = at<PointT>(position, msgTime);
		filtered = filterState;
		return scan;
	}

	template<typename PointT>
	static
	ScanConstPtr<PointT>
	VoxelGridFilter (
		ScanConstPtr<PointT> vcloud,
		double voxel_leaf_size=0.2,
		double measurement_range=3.0)
	{
		ScanPtr<PointT> filteredGridCLoud(new pcl::PointCloud<PointT>);

		assert(voxel_leaf_size>=0.1);
		pcl::VoxelGrid<PointT> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
		voxel_grid_filter.setInputCloud(vcloud);
		voxel_grid_filter.filter(*filteredGridCLoud);

		return filteredGridCLoud;
	}

	template<typename PointT>
	static
	bool save(const pcl::PointCloud<PointT> &vcloud, const std::string &filename)
	{
		return pcl::io::savePCDFileBinary(filename, vcloud);
	}

	// switch for filtering
	bool filtered = false;


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
};

}		/* namespace Vmml */

#endif /* _LIDARSCANBAG2_H_ */
