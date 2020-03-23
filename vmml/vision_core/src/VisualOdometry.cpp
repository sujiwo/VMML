/*
 * VisualOdometry.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#include <opencv2/video/tracking.hpp>
#include "vmml/VisualOdometry.h"
#include "vmml/Triangulation.h"
#include "vmml/utilities.h"


namespace Vmml {


typedef LocalLidarMapper::PointType PointType;
typedef LocalLidarMapper::CloudType CloudType;

const cv::Size optFlowWindowSize(15, 15);
const int maxLevel = 2;
const cv::TermCriteria optFlowStopCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT, 10, 0.03);


VisualOdometry::VisualOdometry(Parameters par) :
	param(par),
	featureGrid(par.bucket_width, par.bucket_height),
	pointsCloud3d(new CloudType)
{
	// XXX: Temporary
	featureDetector=cv::ORB::create(
		maxNumberFeatures,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);
	// NORM_HAMMING is used for binary descriptor, such as ORB and AKAZE
	bfMatch = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}


VisualOdometry::~VisualOdometry()
{
}


/*
bool
VisualOdometry::runMatching (cv::Mat img, const ptime &timestamp, cv::Mat mask)
{
	mCurrentImage = BaseFrame::create(img, param.camera);
	if (frameCounter==0)
		mCurrentImage->setPose(Pose::Identity());

	if (voFeatureTracker.size()>0) {
		uint nextFrameCounter = frameCounter+1, prevFrameCounter = frameCounter-1;
		auto trackedFeatsPoints = voFeatureTracker.trackedFeaturesAtFrame(prevFrameCounter);
		auto &_featureTrackIds = voFeatureTracker.getFeatureTrackIdsAtFrame(prevFrameCounter);
		assert(trackedFeatsPoints.rows==_featureTrackIds.size());
		vector<FeatureTrackList::TrackId> featureTrackIds(_featureTrackIds.begin(), _featureTrackIds.end());
		cv::Mat p1, p0r, status, errOf;

		cv::calcOpticalFlowPyrLK(mAnchorImage->getImage(), mCurrentImage->getImage(), trackedFeatsPoints, p1, status, errOf, optFlowWindowSize, maxLevel, optFlowStopCriteria);
		cv::calcOpticalFlowPyrLK(mCurrentImage->getImage(), mAnchorImage->getImage(), p1, p0r, status, errOf, optFlowWindowSize, maxLevel, optFlowStopCriteria);

		cv::Mat absDiff(cv::abs(trackedFeatsPoints-p0r));

		for (int r=0; r<trackedFeatsPoints.rows; ++r) {
			cv::Point2f pt(p1.row(r));

			// Do track/feature filtering here
			if (absDiff.at<float>(r,0)>=1 or absDiff.at<float>(r,1)>=1)
				continue;
			if (pt.x<0 or pt.x>=mCurrentImage->width() or pt.y<0 or pt.y>=mCurrentImage->height())
				continue;

			// Add tracked point
			auto trackId = featureTrackIds[r];
			voFeatureTracker.addTrackedPoint(frameCounter, trackId, pt);
		}
	}

	if (frameCounter % featureDetectionInterval==0) {
		cv::Mat fMask;
		int prevFrameNumber = frameCounter-1;
		int numOfFeaturesToDetect = maxNumberFeatures;

		// Get feature tracks from previous frame
		if (prevFrameNumber==-1)
			fMask = mask;
		else {
			fMask = voFeatureTracker.createFeatureMask(mask, prevFrameNumber);
			numOfFeaturesToDetect -= voFeatureTracker.getNumberOfFeatureTracksAt(prevFrameNumber);
		}
		featureDetector->setMaxFeatures(numOfFeaturesToDetect);

		mCurrentImage->computeFeatures(featureDetector, fMask);
		// Add all features as new track
		for (int i=0; i<mCurrentImage->numOfKeyPoints(); ++i) {
			FeatureTrack ftNew(frameCounter, mCurrentImage->keypoint(i).pt);
			voFeatureTracker.add(ftNew);
		}
	}

	return true;
}
*/


bool
VisualOdometry::process(cv::Mat img, const ptime &timestamp, cv::Mat mask, bool matchOnly)
{
	bool isMoving;

	if (mAnchorImage==nullptr and mCurrentImage==nullptr) {
		mAnchorImage = BaseFrame::create(img, param.camera);
		mAnchorImage->computeFeatures(featureDetector, mask);
		mAnchorImage->setPose(Pose::Identity());
		mVoTrack.push_back(PoseStamped(Pose::Identity(), timestamp));
		return false;
	}
	if (mCurrentImage!=nullptr)
		mAnchorImage = mCurrentImage;
	mCurrentImage = BaseFrame::create(img, param.camera);
	mCurrentImage->computeFeatures(featureDetector, mask);

	Matcher::matchOpticalFlow(*mAnchorImage, *mCurrentImage, flowMatcherToAnchor, &isMoving);

	if (matchOnly==false and isMoving==true) {

		// Get transformation, and inliers
		vector<Vector3d> points3;
		TTransform motion;
		Matcher::calculateMovement2(
			*mAnchorImage, *mCurrentImage,
			flowMatcherToAnchor, voMatcherToAnchor,
			motion, points3);
		if (voMatcherToAnchor.size()<=10)
			return false;

		Pose pCurrent = mAnchorImage->pose() * motion;
		mCurrentImage->setPose(pCurrent);

		for (auto &_pt: points3) {
			auto pt = pCurrent*_pt.homogeneous();
			pointsCloud3d->push_back(PointType(pt.x(), pt.y(), pt.z()));
		}
		cout << "Found " << points3.size() << " triangulated points" << endl;
	}

	else {
		mCurrentImage->setPose(mAnchorImage->pose());
		voMatcherToAnchor = flowMatcherToAnchor;
	}

	mVoTrack.push_back(PoseStamped(mCurrentImage->pose(), timestamp));

	frameCounter+=1;
	return true;
}


TTransform
VisualOdometry::estimateMotion()
{

}


} /* namespace Vmml */
