/*
 * LocalLidarMapper.h
 *
 *  Created on: Nov 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_LOCALLIDARMAPPER_H_
#define VMML_CORE_LOCALLIDARMAPPER_H_

#include <map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "utilities.h"
#include "Pose.h"
#include "Trajectory.h"


namespace Vmml {

class LocalLidarMapper
{
public:

	typedef pcl::PointXYZ PointType;
	typedef pcl::PointCloud<PointType> CloudType;

	struct Param {
		double
			ndt_res=1.0,
			step_size=0.1,
			trans_eps=0.01;
		int
			max_iter=100;
		double
			voxel_leaf_size=1.0,
			min_scan_range=5.0,
			max_scan_range=130.0,
			min_add_scan_shift=1.0,
			max_submap_size=30.0;
	};

	LocalLidarMapper();
	virtual ~LocalLidarMapper();

	// XXX: Subject to change
	struct ScanProcessLog {
		uint64_t sequence_num;
		ptime timestamp;
		int numOfScanPoints, filteredScanPoints, mapNumOfPoints;
		bool hasConverged = false;
		float
			fitness_score 					= std::numeric_limits<float>::max(),
			transformation_probability 		= std::numeric_limits<float>::max();
		int num_of_iteration;
		Pose poseAtScan=Pose::Identity();
		float shift;
		double submap_size = 0;
		ptime submap_origin_stamp;
		Pose submap_origin_pose=Pose::Identity();
		tduration matchingTime 				= boost::posix_time::seconds(0);

		bool hasScanFrame					= false;
		int64 prevScanFrame					= -1;
		double accum_distance				= 0.0;
		uint32_t submap_id					= 0;
		Twist currentVelocity;

		std::string dump();
	};

	void feed(CloudType::ConstPtr cloud, const ptime &lidarTimestamp, ScanProcessLog &procLog);

	void matching2nd(CloudType::ConstPtr cloud, const TTransform &hint);

	inline const ScanProcessLog& getScanLog(const int64 scanID) const
	{ return scanResults.at(scanID); }

	Trajectory getTrajectory() const;

protected:
	Param param;

	// Need separate NDT instances due to possible different parameters
	pcl::NormalDistributionsTransform<PointType, PointType> mNdt;
	// Need our own voxel grid filter
	pcl::VoxelGrid<PointType> mVoxelGridFilter;

	// Counters
	bool initial_scan_loaded = false;
	uint32_t currentScanId = 0;

	CloudType currentMap, currentSubmap;

	// States
	ptime current_scan_time;
	bool isMapUpdate = true;
	bool hasSubmapIdIncremented = true;
	Pose
		previous_pose = TTransform::Identity(),
		added_pose = TTransform::Identity();
	TTransform
		lastDisplacement = TTransform::Identity(),
		displacementFromOrigin = TTransform::Identity();
	double submap_size = 0;
	uint32_t submap_id = 0;
	ptime submapOriginTimestamp;
	Pose submapOriginPose=Pose::Identity();
	double accum_distance = 0.0;
	int64_t lastScanFrame = -1;

	Trajectory localMapTrack;

	std::map<int64, ScanProcessLog> scanResults;
};

} /* namespace Vmml */

#endif /* VMML_CORE_LOCALLIDARMAPPER_H_ */
