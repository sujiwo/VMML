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
#include "MapBuilder.h"
#include "LidarScanBag.h"
#include "ImageBag.h"


namespace Vmml {

class ImageDatabaseBuilder : public MapBuilder
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

	struct IdbWorkFrame: public BaseFrame
	{
		typedef std::shared_ptr<IdbWorkFrame> Ptr;

		IdbWorkFrame(CloudT::Ptr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam);

		static Ptr create(CloudT::Ptr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam)
		{ return Ptr(new IdbWorkFrame(cl, lstamp, img, istamp, cam)); }

		CloudT::Ptr lidarScan;
		ptime lidarTimestamp;
		ptime imageTimestamp;

		double accumDistance=0.0;
	};

	ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0, const std::string &mapVocabularyPath);
	virtual ~ImageDatabaseBuilder();

	void feed(CloudT::Ptr cloudInp, const ptime& cloudTimestamp, cv::Mat img, const ptime& imageTimestamp);

protected:

	Param mParams;

	IdbWorkFrame::Ptr anchorFrame=nullptr;
	pcl::NormalDistributionsTransform<PointT, PointT> mNdt;
	pcl::VoxelGrid<PointT> mVoxelGridFilter;

	TTransform runMatch(IdbWorkFrame::Ptr targetFrame);
	void addKeyframe(IdbWorkFrame::Ptr keyframe);

};

} /* namespace Vmml */

#endif /* VMML_CORE_IMAGEDATABASEBUILDER_H_ */
