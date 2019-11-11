/*
 * VisualOdometry.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#include "VisualOdometry.h"
#include "Triangulation.h"
#include "utilities.h"


namespace Vmml {

VisualOdometry::VisualOdometry(Parameters par) :
	param(par),
	featureGrid(par.bucket_width, par.bucket_height)
{
	// XXX: Temporary
	featureDetector=cv::ORB::create(
		6000,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);

}


VisualOdometry::~VisualOdometry()
{
}


bool
VisualOdometry::process(cv::Mat img)
{
	mCurrentImage = BaseFrame::create(img, param.camera);
	mCurrentImage->computeFeatures(featureDetector);

	if (mAnchorImage==nullptr) {
		mAnchorImage = mCurrentImage;
		return false;
	}
	Matcher::matchBruteForce(*mAnchorImage, *mCurrentImage, matcherToAnchor);

	// Get transformation, and inliers
	Matcher::PairList f12matchesInliers;
	TTransform motion = Matcher::calculateMovement(*mAnchorImage, *mCurrentImage, matcherToAnchor, f12matchesInliers);
	if (f12matchesInliers.size()<=10)
		return false;

	// Find 3D points
	float parallax;
	std::map<uint, Eigen::Vector3d> trianglPoints;
	TriangulateCV(*mAnchorImage, *mCurrentImage, matcherToAnchor, trianglPoints, &parallax);
	if (trianglPoints.size()<=10)
		return false;

	// Find median
	VectorXx<float> distance=VectorXx<float>::Zero(trianglPoints.size());
	int i=0;
	for (auto &p: trianglPoints) {
		auto p3d = p.second;
		distance[i] = fabs(p3d.x()) + fabs(p3d.y()) + fabs(p3d.z());
		i++;
	}
	float sceneMedian = median(distance);
	if (sceneMedian > param.motion_threshold)
		return false;

	// XXX: Unfinished

	return true;
}


TTransform
VisualOdometry::estimateMotion()
{

}


} /* namespace Vmml */
