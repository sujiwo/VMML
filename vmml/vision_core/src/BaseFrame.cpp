/*
 * BaseFrame.cpp
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include <pcl/filters/frustum_culling.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "BaseFrame.h"
#include "KeyFrame.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


namespace Vmml {


BaseFrame::BaseFrame() :
	mPose(Pose::Identity())
{}


BaseFrame::BaseFrame(cv::Mat img, const CameraPinholeParams &cam, const Pose &p) :
	image(img),
	mPose(p),
	cameraParam(cam)
{}


BaseFrame::BaseFrame(const BaseFrame &c) :
	image(c.image.clone()),
	mPose(c.mPose),
	cameraParam(c.cameraParam),
	poseIsValid(c.poseIsValid),
	fDescriptors(c.fDescriptors.clone()),
	fKeypoints(c.fKeypoints),
	featuresGridIdx(c.featuresGridIdx)
{}


BaseFrame::~BaseFrame() {
	// TODO Auto-generated destructor stub
}


void
BaseFrame::setPose (const g2o::SE3Quat &pq)
{
	auto Q = pq.rotation().inverse();
	auto P = -(Q * pq.translation());
	setPose(P, Q);
}


Eigen::Vector2d
BaseFrame::project (const Eigen::Vector3d &pt3) const
{
	Vector3d ptx = projectionMatrix() * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
}


Eigen::Vector3d
BaseFrame::project3 (const Eigen::Vector3d &pt3) const
{
	return projectionMatrix() * pt3.homogeneous();
}


Vector3d
BaseFrame::transform (const Eigen::Vector3d &pt3) const
{
	Vector4d P = externalParamMatrix4() * pt3.homogeneous();
	return P.hnormalized();
}


poseMatrix4
BaseFrame::externalParamMatrix4 () const
{
	return createExternalParamMatrix4(mPose);
}


Eigen::Matrix4d
BaseFrame::createExternalParamMatrix4(const Pose &ps)
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = ps.orientation().toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*ps.position());
	return ex;
}


BaseFrame::ProjectionMat
BaseFrame::projectionMatrix () const
{
	return cameraParam.toMatrix() * externalParamMatrix4();
}


Vector3d
BaseFrame::normal() const
{
	return externalParamMatrix4().block(0,0,3,3).transpose().col(2).normalized();
}


/*
void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, const cv::Mat &mask)
{
	computeFeatures(fd, fKeypoints, fDescriptors, mask);
}
*/
void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd)
{
	if (image.empty()==true)
		return;
	computeFeatures(fd, fKeypoints, fDescriptors, cameraParam.mask);
	assignKeyPointsToGrid();
}


void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, std::vector<cv::KeyPoint> &kpList, cv::Mat &descriptors, const cv::Mat &mask) const
{
	assert (image.empty() == false);

	// Clear all previous features, to be safe
	kpList.clear();
	descriptors.release();

	// Enforce gray image before computing features
	cv::Mat grayImg;
	if (image.channels()==1)
		grayImg = image;
	else
		cv::cvtColor(image, grayImg, CV_BGR2GRAY, 1);

	fd->detectAndCompute(
		grayImg,
		mask,
		kpList,
		descriptors,
		false);
}


void
BaseFrame::assignKeyPointsToGrid()
{
	for (kpid i=0; i<fKeypoints.size(); ++i) {
		auto keypoint = fKeypoints[i];

		int gX = round(keypoint.pt.x / (cameraParam.width/float(BaseFrame::numberOfGridIn1D))),
		gY = round(keypoint.pt.y / (cameraParam.height/float(BaseFrame::numberOfGridIn1D)));

		featuresGridIdx[gX][gY].push_back(i);
	}
}


vector<kpid>
BaseFrame::getKeyPointsInArea (const float x, const float y, const float windowSize, const int minLevel, const int maxLevel) const
{
	vector<kpid> indices;

    const int nMinCellX = max(0,(int)floor((x-windowSize) / (cameraParam.width/float(BaseFrame::numberOfGridIn1D))));
    if(nMinCellX>=numberOfGridIn1D)
        return indices;

    const int nMaxCellX = min((int)numberOfGridIn1D-1,(int)ceil((x+windowSize) / (cameraParam.width/float(BaseFrame::numberOfGridIn1D))));
    if(nMaxCellX<0)
        return indices;

    const int nMinCellY = max(0,(int)floor((y-windowSize) / (cameraParam.height/float(BaseFrame::numberOfGridIn1D))));
    if(nMinCellY>=numberOfGridIn1D)
        return indices;

    const int nMaxCellY = min((int)numberOfGridIn1D-1,(int)ceil((y+windowSize) / (cameraParam.height/float(BaseFrame::numberOfGridIn1D))));
    if(nMaxCellY<0)
        return indices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            auto vCell = featuresGridIdx[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = fKeypoints[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<windowSize && fabs(disty)<windowSize)
                    indices.push_back(vCell[j]);
            }
        }
    }

	return indices;
}


void
BaseFrame::perturb (PerturbationMode mode,
	bool useRandomMotion,
	double displacement, double rotationAngle)
{
	Vector3d movement;
	switch (mode) {
	case PerturbationMode::Lateral:
		movement = Vector3d(1,0,0); break;
	case PerturbationMode::Vertical:
		movement = Vector3d(0,-1,0); break;
	case PerturbationMode::Longitudinal:
		movement = Vector3d(0,0,1); break;
	}
	movement = displacement * movement;

	mPose = mPose.shift(movement);
}


/*
std::vector<BaseFrame::PointXYI>
BaseFrame::projectLidarScan
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan, const TTransform &lidarToCameraTransform, const CameraPinholeParams &cameraParams)
{
	std::vector<PointXYI> projections;

	// Create fake frame
	BaseFrame frame;
	frame.setPose(lidarToCameraTransform);
	frame.setCameraParam(&cameraParams);

	projections.resize(lidarScan->size());
	int i=0, j=0;
	for (auto it=lidarScan->begin(); it!=lidarScan->end(); ++it) {
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
*/


g2o::SBACam
BaseFrame::forG2O () const
{
	// XXX: Verify this
	g2o::SBACam mycam(orientation(), -orientation().toRotationMatrix()*position());
	mycam.setKcam(cameraParam.fx, cameraParam.fy, cameraParam.cx, cameraParam.cy, 0);

	return mycam;
}


Plane3
BaseFrame::projectionPlane() const
{
	if (cameraParam.fx==0)
		throw runtime_error("Camera parameter is not defined");

	Vector3d centerPt = mPose * Vector3d(0, 0, cameraParam.f());

	Plane3 P(normal(), centerPt);
	P.normalize();
	return P;
}


Eigen::Matrix3d
BaseFrame::FundamentalMatrix(const BaseFrame &F1, const BaseFrame &F2)
{
	if (F1.cameraParam.fx==0 or F2.cameraParam.fx==0)
		throw runtime_error("Camera parameters are not defined");

	TTransform T12 = F1.mPose.inverse() * F2.mPose;
	Matrix3d R = T12.rotation();
	Vector3d t = T12.translation();

	Vector3d A = F1.cameraParam.toMatrix3() * R.transpose() * t;
	Matrix3d C = Matrix3d::Zero();
	C(0,1) = -A[2];
	C(0,2) = A[1];
	C(1,0) = A[2];
	C(1,2) = -A[0];
	C(2,0) = -A[1];
	C(2,1) = A[0];

	return F2.cameraParam.toMatrix3().inverse().transpose() * R * F1.cameraParam.toMatrix3().transpose() * C;
	// XXX: Change to general version using Pseudo-inverse

/*
	Vector3d e0 = F1.project3(F1.position());

	Vector3d e2 = F2.project3(F1.position());
	Matrix3d C = Matrix3d::Zero();
	C(0,1) = -e2[2];
	C(0,2) = e2[1];
	C(1,0) = e2[2];
	C(1,2) = -e2[0];
	C(2,0) = -e2[1];
	C(2,1) = e2[0];

	auto P1inv = pseudoInverse(F1.projectionMatrix());
	Matrix3d F12 = C * F2.projectionMatrix() * P1inv;
	return F12;
*/
}


BaseFrame::Ptr BaseFrame::create(cv::Mat img, const CameraPinholeParams &cam, const Pose &p)
{
	BaseFrame::Ptr bframe(new BaseFrame(img, cam, p));
	return bframe;
}


// Image dimensions
int
BaseFrame::width() const
{
	return cameraParam.width;
}


int
BaseFrame::height() const
{
	return cameraParam.height;
}


void
BaseFrame::projectPointCloud(
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
		const CameraPinholeParams &camera,
		const Pose &cameraPose,
		const double cutDistance,
		MatrixProjectionResult &projRes)
{
	pcl::FrustumCulling<pcl::PointXYZ> frustum;

	frustum.setInputCloud(pointsInWorld);
	frustum.setNearPlaneDistance(0.5);
	frustum.setFarPlaneDistance(cutDistance);
	frustum.setHorizontalFOV(camera.getHorizontalFoV() * 180 / M_PI);
	frustum.setVerticalFOV(camera.getVerticalFoV() * 180 / M_PI);

	Eigen::Matrix4f cam2robot;
	cam2robot <<
			0, 0, 1, 0,
            0,-1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
	Matrix4f poseFilt = cameraPose.cast<float>() * cam2robot;
	frustum.setCameraPose(poseFilt);

	pcl::PointCloud<pcl::PointXYZ> filtered;
	frustum.filter(filtered);

	projRes.resize(filtered.size(), Eigen::NoChange);
	// Must use this matrix data type (dont use auto)
	Matrix<double,3,4> camMat4 = camera.toMatrix() * createExternalParamMatrix4(cameraPose);
	for (uint64 i=0; i<filtered.size(); ++i) {
		auto pp = filtered.at(i);
		Vector4d pv(pp.x, pp.y, pp.z, 1);
		Vector3d pj = camMat4 * pv;
		pj /= pj[2];
		projRes.row(i) = pj.hnormalized();
	}
}


/*
 * Project point cloud in World Coordinate (eg. PCL Map) to this frame
 */
void
BaseFrame::projectPointCloud(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
	const double cutDistance,
	MatrixProjectionResult &projRes) const
{
	return projectPointCloud(pointsInWorld, cameraParam, mPose, cutDistance, projRes);
}


/*
 * Project/Render point cloud in World Coordinate using image of this frame
 */
cv::Mat
BaseFrame::projectPointCloud(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
	const double cutDistance) const
{
	cv::Mat frameImage = this->image.clone();
	MatrixProjectionResult projRes;

//	projectPointCloud(pointsInWorld, cutDistance, projRes);
	projectPointCloud(pointsInWorld, cameraParam, mPose, cutDistance, projRes);
	for (int r=0; r<projRes.rows(); ++r) {
		cv::Point2f p2f (projRes(r,0), projRes(r,1));
		cv::circle(frameImage, p2f, 2, cv::Scalar(0,0,255));
	}

	return frameImage;
}


Eigen::Vector3d
BaseFrame::keypointn (kpid k) const
{
	Vector3d v = keypointh(k);
	return cameraParam.toMatrix3().inverse() * v;
}


void
BaseFrame::extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<cv::KeyPoint> &keypointsInMask, cv::Mat &descriptorsInMask) const
{
	vector<uint32_t> kpIds;

	extractKeypointsAndFeatures(mask, kpIds);
	keypointsInMask.resize(kpIds.size());
	descriptorsInMask = cv::Mat(kpIds.size(), fDescriptors.cols, fDescriptors.type());

	for (int i=0; i<kpIds.size(); ++i) {
		auto id = kpIds[i];
		keypointsInMask[i] = fKeypoints.at(id);
		descriptorsInMask.row(i) = fDescriptors.row(id);
	}
}


/*
 * XXX: Unfinished
 */
void
BaseFrame::extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<uint32_t> &keypointIds) const
{
	keypointIds.clear();
	for (int i=0; i<numOfKeyPoints(); ++i) {
		cv::Point2i kp = fKeypoints.at(i).pt;
		if (mask.at<int>(kp)==0)
			continue;
		keypointIds.push_back(static_cast<uint32_t>(i));
	}
}


g2o::Sim3
BaseFrame::toSim3() const
{
//	g2o::
}


g2o::SE3Quat
BaseFrame::toSE3Quat() const
{
	auto extMat = createExternalParamMatrix4(mPose);
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


/*
 * Associate image features with depth from Lidar
 */
void BaseFrame::associateToLidarScans
	(const pcl::PointCloud<pcl::PointXYZ> &lidarScan,
	const TTransform &lidarToCameraTransform,
	std::map<uint32_t, uint32_t> imageFeaturesToLidar,
	pcl::PointCloud<pcl::PointXYZ> *associationResult)
const
{
	// XXX: Need to create versions that detect invalid projection
	auto lidarProjections = BaseFrame::projectLidarScan(lidarScan, lidarToCameraTransform, cameraParam);
	pcl::KdTreeFLANN<pcl::PointXY> flannel;

	pcl::PointCloud<pcl::PointXY>::Ptr cloudProjs(new pcl::PointCloud<pcl::PointXY>);
	cloudProjs->reserve(lidarProjections.size());
	for (auto &pt: lidarProjections) {
		cloudProjs->push_back(pcl::PointXY{pt.x, pt.y});
	}
	flannel.setInputCloud(cloudProjs);

#define _CoC_ 0.58

	vector<int> index1(3);
	vector<float> dist1(3);
	for (uint32_t ix=0; ix<fKeypoints.size(); ++ix) {
		auto &kp = fKeypoints[ix];
		// Unfinished
		pcl::PointXY pt{kp.pt.x, kp.pt.y};
		index1.clear();
		dist1.clear();
		if (flannel.nearestKSearch(pt, 3, index1, dist1) > 0) {
			for (int x=0; x<index1.size(); x++) {
				if (dist1[x] > 2*_CoC_)
					return;
				else {
					imageFeaturesToLidar.insert(make_pair(ix, index1[x]));
					if (associationResult != NULL) {
						associationResult->push_back(lidarScan[index1[x]]);
					}
				}
			}
		}
	}

}


void
BucketFeature::assignFeatures(const int imageWidth, const int imageHeight, const std::vector<cv::KeyPoint> &kpList)
{
	for (kpid id=0; id<kpList.size(); id++) {
		auto keypoint = kpList.at(id);

		int gX = round(keypoint.pt.x / (imageWidth/float(BaseFrame::numberOfGridIn1D))),
		gY = round(keypoint.pt.y / (imageHeight/float(BaseFrame::numberOfGridIn1D)));

		at(gX, gY).push_back(id);
	}
}


void
BaseFrame::computeBoW (DBoW2::BowVector &words, DBoW2::FeatureVector &featureVec, const ORBVocabulary &oVoc) const
{
	words.clear();
	featureVec.clear();

	auto frameDescVectors = KeyFrame::toDescriptorVector(fDescriptors);
	oVoc.transform(frameDescVectors, words, featureVec, 4);
}

} /* namespace Vmml */
