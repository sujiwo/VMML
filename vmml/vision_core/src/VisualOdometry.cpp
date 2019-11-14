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
VisualOdometry::process(cv::Mat img, const ptime &timestamp)
{
	mCurrentImage = BaseFrame::create(img, param.camera);
	mCurrentImage->computeFeatures(featureDetector);

	if (mAnchorImage==nullptr) {
		mAnchorImage = mCurrentImage;
		mAnchorImage->setPose(Pose::Identity());
		mVoTrack.push_back(PoseStamped(Pose::Identity(), timestamp));
		return false;
	}
	Matcher::matchBruteForce(*mAnchorImage, *mCurrentImage, matcherToAnchor);

	// Get transformation, and inliers
	TTransform motion = Matcher::calculateMovement(*mAnchorImage, *mCurrentImage, matcherToAnchor, matcherToAnchor);
	if (matcherToAnchor.size()<=10)
		return false;

	Pose pCurrent = mAnchorImage->pose() * motion;
	mCurrentImage->setPose(pCurrent);
	mVoTrack.push_back(PoseStamped(mCurrentImage->pose(), timestamp));

	mAnchorImage = mCurrentImage;

	return true;
}


TTransform
VisualOdometry::estimateMotion()
{

}


} /* namespace Vmml */
