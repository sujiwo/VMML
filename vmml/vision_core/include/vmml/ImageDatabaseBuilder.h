/*
 * ImageDatabaseBuilder.h
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_IMAGEDATABASEBUILDER_H_
#define VMML_CORE_IMAGEDATABASEBUILDER_H_


#include <string>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include "SimpleMapBuilder.h"
#include "Trajectory.h"
#include "Matcher.h"


namespace Vmml {

class ImageDatabaseBuilder : public SimpleMapBuilder
{
public:

	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> CloudT;

	struct Param
	{
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
			min_linear_move=1.0,
			min_angular_move=0.087,	// 5 degrees
			max_submap_size=30.0;
	};

	/*
	 * Pose of this frame is lidar pose
	 */
	struct IdbWorkFrame: public BaseFrame
	{
		typedef std::shared_ptr<IdbWorkFrame> Ptr;

		IdbWorkFrame(CloudT::ConstPtr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam);

		static Ptr create(CloudT::ConstPtr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam)
		{ return Ptr(new IdbWorkFrame(cl, lstamp, img, istamp, cam)); }

		CloudT::ConstPtr lidarScan;
		ptime lidarTimestamp;
		ptime imageTimestamp;
		kfid keyframeRel=0;
		bool isKeyFrame = false;

		double accumDistance=0.0;

		Matcher::PairList featureMatchesFromLastAnchor;
	};

	ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0);
	virtual ~ImageDatabaseBuilder();

	bool feed(CloudT::ConstPtr cloudInp, const ptime& cloudTimestamp, cv::Mat img, const ptime& imageTimestamp);

	const Trajectory& getTrajectory() const
	{ return rigTrack; }

	void setTranformationFromLidarToCamera (const TTransform &tlc)
	{ lidarToCamera = tlc; }

	const IdbWorkFrame::Ptr& getLastFrame() const
	{ return lastFrame; }

	inline const TTransform& getLidarToCameraTransform() const
	{ return lidarToCamera; }

protected:

	Param mParams;

	// transformation from lidar to camera
	TTransform lidarToCamera = TTransform::Identity();

	IdbWorkFrame::Ptr
		anchorFrame=nullptr,
		lastFrame = nullptr;

	pcl::NormalDistributionsTransform<PointT, PointT> mNdt;
	pcl::VoxelGrid<PointT> mVoxelGridFilter;

	TTransform runNdtMatch(IdbWorkFrame::Ptr frame1, IdbWorkFrame::Ptr frame2);
	void addKeyframe(IdbWorkFrame::Ptr keyframe);
	void buildVisionMap(IdbWorkFrame::Ptr frame1, IdbWorkFrame::Ptr frame2);

	// Lidar trajectory
	Trajectory rigTrack;
};

} /* namespace Vmml */

#endif /* VMML_CORE_IMAGEDATABASEBUILDER_H_ */
