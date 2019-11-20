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


class BucketFeature : public Grid<std::vector<kpid>>
{
public:
	BucketFeature(int numCellHorizontal, int numCellVertical) :
		Grid<std::vector<kpid>>(numCellHorizontal, numCellVertical)
	{}

	inline kpid operator()(const int x, const int y, const int z)
	{ return mGrid[y][x]->at(z); }

	void assignFeatures(const int imageWidth, const int imageHeight, const std::vector<cv::KeyPoint> &kp);
};


class BaseFrame
{
public:

	typedef std::shared_ptr<BaseFrame> Ptr;
	typedef std::shared_ptr<BaseFrame const> ConstPtr;

	friend class BucketFeature;

	BaseFrame();
	BaseFrame(cv::Mat img, const CameraPinholeParams &cam, const Pose &p=Pose::Identity());
	BaseFrame(const BaseFrame &copy);
	virtual ~BaseFrame();

	static
	Ptr create(cv::Mat img, const CameraPinholeParams &cam, const Pose &p=Pose::Identity());

	const Pose& pose() const
	{ return mPose; }

	void setPose (const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
	{ mPose = Pose::from_Pos_Quat(p, q); }

	void setPose (const g2o::SE3Quat &pq);

	void setPose (const Pose &p)
	{ mPose = p; }

	Eigen::Vector3d position() const
	{ return mPose.position(); }

	Eigen::Quaterniond orientation() const
	{ return mPose.orientation(); }

	// Project to 2D
	Eigen::Vector2d project (const Eigen::Vector3d &pt3) const;

	// Similar to above, but with third coordinate value as F
	Eigen::Vector3d project3 (const Eigen::Vector3d &pt3) const;

	// Transform a point in World coordinate to Frame-centric one
	Eigen::Vector3d transform (const Eigen::Vector3d &pt3) const;

	typedef Eigen::Matrix<double,Eigen::Dynamic,2> MatrixProjectionResult;

	/*
	 * Project point cloud in World Coordinate (eg. PCL Map) using this frame's pose
	 */
	void projectPointCloud(
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
		const double cutDistance,
		MatrixProjectionResult &projRes) const;

	static void
	projectPointCloud(
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
		const CameraPinholeParams &camera,
		const Pose &cameraPose,
		const double cutDistance,
		MatrixProjectionResult &res);

	/*
	 * Project/Render point cloud in World Coordinate using image of this frame
	 */
	cv::Mat projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
		const double cutDistance) const;

	void setCameraParam(const CameraPinholeParams *c)
	{ cameraParam = *c; }

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

	void computeFeatures (cv::Ptr<cv::FeatureDetector> fd);

//	void computeFeatures (cv::Ptr<cv::FeatureDetector> fd, const cv::Mat &mask=cv::Mat());

	void computeFeatures (cv::Ptr<cv::FeatureDetector> fd, std::vector<cv::KeyPoint> &kpList, cv::Mat &descriptors, const cv::Mat &mask) const;

	/*
	 * Extract keypoints (and their descriptors) according to mask.
	 * A keypoint is included when the color in the mask is not zero
	 */
	void extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<uint32_t> &keypointIds) const;
	void extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<cv::KeyPoint> &keypointsInMask, cv::Mat &descriptorsInMask) const;

	inline const cv::Mat descriptor(kpid r) const
	{ return fDescriptors.row(r).clone(); }

	const cv::KeyPoint keypoint(kpid k) const
	{ return fKeypoints.at(k); }

	inline Eigen::Vector2d keypointv(kpid k) const
	{
		const cv::KeyPoint& kp = fKeypoints.at(k);
		return Eigen::Vector2d(kp.pt.x, kp.pt.y);
	}

	inline Eigen::Vector3d keypointh(kpid k) const
	{
		return keypointv(k).homogeneous();
	}

	/*
	 * Keypoint in normalized image coordinate
	 */
	Eigen::Vector3d keypointn (kpid k) const;

	std::vector<kpid> getKeyPointsInArea (const float x, const float y, const float windowSize, const int minLevel=-1, const int maxLevel=-1) const;

	cv::Mat allDescriptors() const
	{ return fDescriptors; }

	const std::vector<cv::KeyPoint>& allKeypoints() const
	{ return fKeypoints; }

	int numOfKeypoints() const
	{ return fKeypoints.size(); }

	int numOfKeyPoints() const
	{ return fKeypoints.size(); }

	cv::Mat getImage() const
	{ return image; }

	enum PerturbationMode {
		Lateral,
		Longitudinal,
		Vertical
	};

	void perturb (
		PerturbationMode h,
		bool useRandomMotion,
		double displacement=0, double rotationAngle=0);


	struct PointXYI : public pcl::PointXY
	{
		inline PointXYI() {}

		inline PointXYI(const float &_x, const float &_y, const unsigned int &_i=0) :
			i(_i)
		{ x=_x; y=_y; i=_i; }

		unsigned int i;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};


/*
	static std::vector<PointXYI>
	projectLidarScan
	(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan,
	const TTransform &lidarToCameraTransform,
	const CameraPinholeParams &cameraParams);
*/

	/*
	 * Project lidar scans to image plane, without considering mapping to original lidar scans
	 */
	template<class PointT>
	static std::vector<PointXYI>
	projectLidarScan
	(const pcl::PointCloud<PointT> &lidarScan,
	const TTransform &lidarToCameraTransform,
	const CameraPinholeParams &cameraParams);

	template<class PointT>
	std::vector<PointXYI>
	projectLidarScan
		(const pcl::PointCloud<PointT> &lidarScan,
		const TTransform &lidarToCameraTransform)
	const
	{ return projectLidarScan(lidarScan, lidarToCameraTransform, cameraParam); }

	/*
	 * Project lidar scans to image plane, storing mapping from each point in original lidar to its projection.
	 * Filter those points who fall outside image rectangle
	 */
	typedef std::map<uint32_t,PointXYI> ProjectionMap;

	template<class PointT>
	static void
	projectLidarScan
	(const pcl::PointCloud<PointT> &lidarScan, const TTransform &lidarToCameraTransform, const CameraPinholeParams &cameraParams, ProjectionMap &projmap);

	g2o::SBACam forG2O () const;
	g2o::Sim3 toSim3() const;
	g2o::SE3Quat toSE3Quat() const;

	Plane3 projectionPlane() const;

	// Fundamental matrix from F1 to F2
	static
	Eigen::Matrix3d
	FundamentalMatrix(const BaseFrame &F1, const BaseFrame &F2);

	// Image dimensions
	int width() const;
	int height() const;

	bool hasPose() const
	{ return poseIsValid; }

	/*
	 * Associate image features with depth from Lidar
	 */
	void associateToLidarScans
		(const pcl::PointCloud<pcl::PointXYZ> &lidarScan,
		const TTransform &lidarToCameraTransform,
		std::map<uint32_t, uint32_t> imageFeaturesToLidar,
		pcl::PointCloud<pcl::PointXYZ> *associationResult=NULL)
	const;

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

	friend class Matcher;

	// KeyPoints are assigned to cells in a grid
	static const int numberOfGridIn1D = 10;
	void assignKeyPointsToGrid();
	std::vector<kpid> featuresGridIdx[numberOfGridIn1D][numberOfGridIn1D];
};


template<class PointT>
std::vector<BaseFrame::PointXYI>
BaseFrame::projectLidarScan
(const pcl::PointCloud<PointT> &lidarScan,
	const TTransform &lidarToCameraTransform,
	const CameraPinholeParams &cameraParams)
{
	std::vector<PointXYI> projections;

	// Create fake frame
	BaseFrame frame(cv::Mat(cv::Size(cameraParams.width, cameraParams.height), CV_8UC1),
		cameraParams,
		lidarToCameraTransform);

	projections.resize(lidarScan.size());
	int i=0, j=0;
	for (auto it=lidarScan.begin(); it!=lidarScan.end(); ++it) {
		auto &pts = *it;
		Vector3d pt3d (pts.x, pts.y, pts.z);

		auto p3cam = frame.externalParamMatrix4() * pt3d.homogeneous();
		if (p3cam.z() >= 0) {
			auto p2d = frame.project(pt3d);
			projections[i] = PointXYI(p2d.x(), p2d.y(), j);
			++i;
		}
		j++;
	}

	return projections;
}


template<class PointT>
void
BaseFrame::projectLidarScan
(const pcl::PointCloud<PointT> &lidarScan, const TTransform &lidarToCameraTransform, const CameraPinholeParams &cameraParams, ProjectionMap &projmap)
{
	projmap.clear();

	// Create fake frame
	BaseFrame frame;
	frame.setPose(lidarToCameraTransform);
	frame.setCameraParam(&cameraParams);

	uint j=0;
	for (auto it=lidarScan.begin(); it!=lidarScan.end(); ++it) {
		auto &pts = *it;
		Vector3d pt3d (pts.x, pts.y, pts.z);

		auto p3cam = frame.externalParamMatrix4() * pt3d.homogeneous();
		if (p3cam.z() >= 0) {
			auto p2d = frame.project(pt3d);
			if ((p2d.x()>=0 and p2d.x()<cameraParams.width) and (p2d.y()>=0 and p2d.y()<cameraParams.height)) {
				projmap.insert(
					std::make_pair(
						j,
						BaseFrame::PointXYI(
							p2d.x(),
							p2d.y()
						)));
			}
		}
		j++;
	}

}



} /* namespace Vmml */

#endif /* VMML_CORE_BASEFRAME_H_ */
