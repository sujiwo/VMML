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
#include <velodyne_pointcloud/point_types.h>
#include "RandomAccessBag.h"


namespace Vmml {

template<typename PointT>
using ScanPtr = boost::shared_ptr<pcl::PointCloud<PointT>>;

template<typename PointT>
using ScanConstPtr = boost::shared_ptr<const pcl::PointCloud<PointT>>;

/*
 * Abstraction of lidar point cloud stored in ROS Bag
 */
class LidarScanBag : public RandomAccessBag
{
public:

	typedef std::shared_ptr<LidarScanBag> Ptr;

	LidarScanBag(
		rosbag::Bag const &bag,
		const std::string &topic) :
			RandomAccessBag(bag, topic)
	{
		if (messageType() != "sensor_msgs/PointCloud2")
			throw std::invalid_argument("Requested topic is not of type sensor_msgs/PointCloud2");
	}

	template<typename PointT>
	ScanConstPtr<PointT>
	at(int position, boost::posix_time::ptime *msgTime=nullptr)
	{
		auto msgP = RandomAccessBag::at<sensor_msgs::PointCloud2>(position);
		if (msgTime!=nullptr)
			*msgTime = msgP->header.stamp.toBoost();
		return convertMessage<PointT>(*msgP);
	}

	template<typename PointT=pcl::PointXYZ>
	ScanConstPtr<PointT>
	getFiltered(int position, boost::posix_time::ptime *msgTime=nullptr)
	{
		auto filterState = filtered;
		filtered = true;
		auto scan = at<PointT>(position, msgTime);
		filtered = filterState;
		return scan;
	}

	template<typename PointT=pcl::PointXYZ>
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

	template<typename PointT>
	ScanConstPtr<PointT>
	convertMessage(const sensor_msgs::PointCloud2 &bagmsg)
	{
		ScanPtr<PointT> cldRet(new pcl::PointCloud<PointT>);
		pcl::fromROSMsg(bagmsg, *cldRet);
		return cldRet;
	}
};

}		/* namespace Vmml */

#endif /* _LIDARSCANBAG2_H_ */
