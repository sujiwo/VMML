/*
 * BaseFrame.h
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_BASEFRAME_H_
#define VMML_CORE_BASEFRAME_H_

#include <memory>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/sba/types_sba.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sim3/sim3.h>

#include "CameraPinholeParams.h"
#include "utilities.h"
#include "Pose.h"


namespace Vmml {

inline Eigen::Vector2d cv2eigen (const cv::Point2f &p)
{ return Eigen::Vector2d (p.x, p.y); }


class BaseFrame
{
public:
	BaseFrame();
	virtual ~BaseFrame();

	typedef std::shared_ptr<BaseFrame> Ptr;
	typedef std::shared_ptr<BaseFrame const> ConstPtr;

	static
	Ptr create(cv::Mat img, const CameraPinholeParams &cam, const Pose &p=Pose::Identity());

	const Pose& pose() const
	{ return mPose; }

	Eigen::Vector3d position() const
	{ return mPose.position(); }

	Eigen::Quaterniond orientation() const
	{ return mPose.orientation(); }

	void setPose (const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
	{ mPose = Pose::from_Pos_Quat(p, q); }

	void setPose (const g2o::SE3Quat &pq);

	void setPose (const Pose &p)
	{ mPose = p; }

	inline void setCameraParam(const CameraPinholeParams &c)
	{ cameraParam = c; }

	CameraPinholeParams getCameraParameters() const
	{ return cameraParam; }

	/*
	 * This matrix transforms points in World Coordinate to Frame-centric coordinate
	 */
	Eigen::Matrix4d externalParamMatrix4 () const;

	/*
	 * Similar to the above function, only for general pose
	 */
	static Eigen::Matrix4d createExternalParamMatrix4(const Pose &ps);

	typedef Eigen::Matrix<double,3,4> ProjectionMat;

	ProjectionMat projectionMatrix () const;

	/*
	 * Normal vector; Or, Z-Axis of this frame
	 */
	Eigen::Vector3d normal() const;

protected:
	cv::Mat image;

	/*
	 * A word on pose: this variable stores frame's pose in world coordinate system,
	 * with X->left, Y->bottom, Z->front
	 */
	Pose mPose = Pose::Identity();
	bool poseIsValid = true;

	CameraPinholeParams cameraParam;

	/*
	 * =================
	 * 2D Image Features
	 */

	cv::Mat fDescriptors;

	std::vector<cv::KeyPoint> fKeypoints;

};

} /* namespace Vmml */

#endif /* VMML_CORE_BASEFRAME_H_ */
