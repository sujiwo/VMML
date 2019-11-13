/*
 * LocalLidarMapper.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: sujiwo
 */

#include <utility>
#include "LocalLidarMapper.h"

using namespace Eigen;
using namespace std;


namespace Vmml {

LocalLidarMapper::LocalLidarMapper()
{
}


LocalLidarMapper::~LocalLidarMapper()
{
}


void
LocalLidarMapper::feed(CloudType::ConstPtr newScan, const ptime &messageTime, ScanProcessLog &procLog)
{
	ScanProcessLog feedResult;

	current_scan_time = messageTime;

	feedResult.timestamp = messageTime;

	try {
		if (currentScanId==0) {
			feedResult.hasScanFrame = true;
			lastScanFrame = currentScanId;
		}

		// Add initial point cloud to velodyne_map
		if (initial_scan_loaded==false) {
			currentMap += *newScan;
			initial_scan_loaded = true;

			throw feedResult;
			return;
		}

		// Apply voxel grid filter.
		// The result of this filter will be used to match current submap
		CloudType::Ptr filtered_scan_ptr (new CloudType);
		mVoxelGridFilter.setInputCloud(newScan);
		mVoxelGridFilter.filter(*filtered_scan_ptr);

		CloudType::Ptr map_ptr(new CloudType(currentMap));

		mNdt.setInputSource(filtered_scan_ptr);

		if (isMapUpdate==true) {
			mNdt.setInputTarget(map_ptr);
			isMapUpdate = false;
		}

		// Guess Pose
		Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
		TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
		Pose guessPose = previous_pose * guessDisplacement;

		CloudType::Ptr output_cloud(new CloudType);

		ptime trun1 = getCurrentTime();
		mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
		ptime trun2 = getCurrentTime();
		feedResult.matchingTime = trun2 - trun1;

		TTransform t_localizer = mNdt.getFinalTransformation().cast<double>();

		CloudType::Ptr transformed_scan_ptr(new CloudType);
		pcl::transformPointCloud(*newScan, *transformed_scan_ptr, t_localizer);

		Pose current_pose = t_localizer;

		if (hasSubmapIdIncremented==true) {
			submapOriginTimestamp = messageTime;
			submapOriginPose = current_pose;
			hasSubmapIdIncremented = false;
		}

		// Calculate the displacement (or offset) (current_pose - previous_pose)
		auto prevFrameLog = getScanLog(currentScanId-1);
		lastDisplacement = previous_pose.inverse() * current_pose;
		double linDisplacement, anglDisplacement;
		previous_pose.displacement(current_pose, linDisplacement, anglDisplacement);
		feedResult.currentVelocity = Twist(prevFrameLog.poseAtScan, current_pose, toSeconds(messageTime-prevFrameLog.timestamp));

		// Calculate shift (in X-Y plane only)
		double shift = Vector2d(current_pose.x()-added_pose.x(), current_pose.y()-added_pose.y()).norm();
		// Update the map when horizontal shift is larger than minimum range
		if (shift >= param.min_add_scan_shift) {

			auto lastMapShift = added_pose.inverse() * current_pose;

			// add to pose graph
			accum_distance += lastMapShift.translation().norm();
			feedResult.accum_distance = accum_distance;

			// tell parent to create new scanframe
			feedResult.hasScanFrame = true;
			feedResult.prevScanFrame = lastScanFrame;
			lastScanFrame = currentScanId;

			submap_size += shift;
			// Delay map increment until 2nd matching
/*
			currentMap += *transformed_scan_ptr;
			currentSubmap += *transformed_scan_ptr;
*/
			added_pose = current_pose;
			isMapUpdate = true;
		}

		// Update position
		previous_pose = current_pose;
		feedResult.poseAtScan = current_pose;
		feedResult.submap_id = submap_id;

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

		localMapTrack.push_back(PoseStamped(current_pose, messageTime));
		feedResult.fitness_score = mNdt.getFitnessScore();
		feedResult.num_of_iteration = mNdt.getFinalNumIteration();
		feedResult.hasConverged = mNdt.hasConverged();
		feedResult.transformation_probability = mNdt.getTransformationProbability();

		throw feedResult;

//	scanResults[scanId] = feedResult;
	} catch (const ScanProcessLog &lg) {
		scanResults.insert(make_pair(currentScanId, lg));
		procLog = lg;
		currentScanId++;
	}
}


void
LocalLidarMapper::matching2nd(CloudType::ConstPtr cloud, const TTransform &transFromLastFrame)
{
	// XXX: debug
	double
		r1=transFromLastFrame.roll(),
		p1=transFromLastFrame.pitch(),
		y1=transFromLastFrame.yaw();

	CloudType::Ptr map_ptr(new CloudType(currentMap));

	// Apply voxel grid filter.
	// The result of this filter will be used to match current submap
	CloudType::Ptr filtered_scan_ptr (new CloudType);
	mVoxelGridFilter.setInputCloud(cloud);
	mVoxelGridFilter.filter(*filtered_scan_ptr);

	mNdt.setInputTarget(map_ptr);
	mNdt.setInputSource(filtered_scan_ptr);

	// Guess pose
	auto &lastLog = scanResults.at(currentScanId-1);
	auto prevFrameLog = scanResults.at(lastLog.prevScanFrame);
	Pose fpx = prevFrameLog.poseAtScan * transFromLastFrame;

	double
		r2=lastLog.poseAtScan.roll(),
		p2=lastLog.poseAtScan.pitch(),
		y2=lastLog.poseAtScan.yaw();

	CloudType::Ptr output_cloud(new CloudType);
	ptime trun1 = getCurrentTime();
	mNdt.align(*output_cloud, fpx.matrix().cast<float>());
	ptime trun2 = getCurrentTime();
	lastLog.matchingTime += (trun2 - trun1);

	TTransform t_localizer_final = mNdt.getFinalTransformation().cast<double>();
	CloudType::Ptr transformed_scan_ptr(new CloudType);
	pcl::transformPointCloud(*cloud, *transformed_scan_ptr, t_localizer_final);

	Pose current_pose = t_localizer_final;

	// Map increment
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
	// XXX: stub
}


} /* namespace Vmml */
