/*
 * LocalLidarMapper.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: sujiwo
 */

#include <utility>
#include <iostream>
#include "LocalLidarMapper.h"

using namespace Eigen;
using namespace std;


namespace Vmml {


const LocalLidarMapper::Param defaultParam;


LocalLidarMapper::LocalLidarMapper() :
	LocalLidarMapper(defaultParam)
{}


LocalLidarMapper::LocalLidarMapper(Param p) :
	param(p)
{
	// Parameter set
	mNdt.setResolution(param.ndt_res);
	mNdt.setStepSize(param.step_size);
	mNdt.setTransformationEpsilon(param.trans_eps);
	mNdt.setMaximumIterations(param.max_iter);

	mVoxelGridFilter.setLeafSize(param.voxel_leaf_size, param.voxel_leaf_size, param.voxel_leaf_size);
}


LocalLidarMapper::~LocalLidarMapper()
{
}


TTransform
LocalLidarMapper::matching1st(CloudType::ConstPtr cloud, const ptime &lidarTimestamp)
{
	ScanProcessLog feedResult;
	feedResult.sequence_num = currentScanId;
	feedResult.timestamp = lidarTimestamp;
	current_scan_time = lidarTimestamp;

	try {
		if (currentScanId==0) {
			feedResult.hasScanFrame = true;
			lastScanFrame = currentScanId;
		}

		// Add initial point cloud to velodyne_map
		if (initial_scan_loaded==false) {
			currentMap += *cloud;
			initial_scan_loaded = true;
			feedResult.poseAtScan = Pose::Identity();
			throw feedResult;
		}

		// Apply voxel grid filter.
		// The result will be used to match current submap
		filteredScanPtr.reset(new CloudType);
		mVoxelGridFilter.setInputCloud(cloud);
		mVoxelGridFilter.filter(*filteredScanPtr);

		CloudType::Ptr map_ptr(new CloudType(currentMap));

		mNdt.setInputSource(filteredScanPtr);

		if (isMapUpdate==true) {
			mNdt.setInputTarget(map_ptr);
			isMapUpdate = false;
		}

		// Guess Pose
		Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
		TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
		Pose guessPose = previous_pose * guessDisplacement;

		CloudType::Ptr output_cloud(new CloudType);

		cout << "First NDT matching in frame #" << currentScanId << endl;
		ptime trun1 = getCurrentTime();
		mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
		ptime trun2 = getCurrentTime();
		feedResult.matchingTime = trun2 - trun1;
		Pose current_pose = mNdt.getFinalTransformation().cast<double>();

		if (hasSubmapIdIncremented==true) {
			submapOriginTimestamp = lidarTimestamp;
			submapOriginPose = current_pose;
			hasSubmapIdIncremented = false;
		}

		// Calculate shift (in X-Y plane only) from last scan frame
		auto lastFrameLog = scanResults.at(lastScanFrame);
		const double shift = Vector2d(current_pose.x()-lastFrameLog.poseAtScan.x(), current_pose.y()-lastFrameLog.poseAtScan.y()).norm();
		// Update the map when horizontal shift is larger than minimum range
		if (shift >= param.min_add_scan_shift) {
			// tell parent to create new scanframe
			feedResult.hasScanFrame = true;
			feedResult.prevScanFrame = lastScanFrame;
			lastScanFrame = currentScanId;
		}

		feedResult.poseAtScan = current_pose;
		feedResult.submap_id = submap_id;
		feedResult.fitness_score = mNdt.getFitnessScore();
		feedResult.num_of_iteration = mNdt.getFinalNumIteration();
		feedResult.hasConverged = mNdt.hasConverged();
		feedResult.transformation_probability = mNdt.getTransformationProbability();
		throw feedResult;

	} catch (const ScanProcessLog &lg) {
		scanResults.insert(make_pair(currentScanId, lg));
		currentScanId++;
		return lg.poseAtScan;
	}
}


Pose
LocalLidarMapper::matching2nd(CloudType::ConstPtr cloud, const TTransform &guessPose)
{
	CloudType::Ptr map_ptr(new CloudType(currentMap));

	mNdt.setInputTarget(map_ptr);
	mNdt.setInputSource(filteredScanPtr);

	// Guess pose
	auto &lastLog = scanResults.at(currentScanId-1);
	auto prevFrameLog = scanResults.at(lastLog.prevScanFrame);

	cout << "Second NDT matching in frame #" << currentScanId-1 << endl;
	CloudType::Ptr output_cloud(new CloudType);
	ptime trun1 = getCurrentTime();
	mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
	ptime trun2 = getCurrentTime();
	lastLog.matchingTime += (trun2 - trun1);

	TTransform t_localizer_final = mNdt.getFinalTransformation().cast<double>();
	CloudType::Ptr transformed_scan_ptr(new CloudType);
	pcl::transformPointCloud(*cloud, *transformed_scan_ptr, t_localizer_final);

	Pose current_pose = t_localizer_final;

	// Calculate the displacement (or offset) (current_pose - previous_pose)
	lastDisplacement = prevFrameLog.poseAtScan.inverse() * current_pose;
	lastLog.currentVelocity = Twist(lastDisplacement, toSeconds(lastLog.timestamp-prevFrameLog.timestamp));

	if (lastLog.hasScanFrame) {
		auto lastMapShift = added_pose.inverse() * current_pose;

		// add to pose graph
		accum_distance += lastMapShift.translation().norm();
		lastLog.accum_distance = accum_distance;

		submap_size += Vector2d(current_pose.x()-added_pose.x(), current_pose.y()-added_pose.y()).norm();;
		added_pose = current_pose;
		currentMap += *transformed_scan_ptr;
		currentSubmap += *transformed_scan_ptr;
		isMapUpdate = true;
	}

	// Update position
	previous_pose = current_pose;
	lastLog.poseAtScan = current_pose;

	// Output submap file after a certain threshold
	if (submap_size >= param.max_submap_size) {
		if (currentSubmap.size() != 0) {

//			outputCurrentSubmap();

			currentMap = currentSubmap;
			currentSubmap.clear();
			submap_size = 0;
		}

		submap_id++;
		hasSubmapIdIncremented = true;
	}

	return current_pose;
}


Trajectory
LocalLidarMapper::getTrajectory() const
{
	Trajectory ldTrack;
	for (const auto &fr: scanResults) {
		PoseStamped pstamp(fr.second.poseAtScan, fr.second.timestamp);
		ldTrack.push_back(pstamp);
	}

	return ldTrack;
}


} /* namespace Vmml */
