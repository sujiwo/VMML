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


VisualOdometry::VisualOdometry(Parameters par) :
	param(par),
	featureGrid(par.bucket_width, par.bucket_height),
	points3d(new CloudType)
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
}


VisualOdometry::~VisualOdometry()
{
}


//cv::Ptr<cv::BFMatcher> featureBfMatcher = cv::BFMatcher::create(cv::NORM_HAMMING);


/*
bool
VisualOdometry::runMatching (cv::Mat img, const ptime &timestamp, cv::Mat mask)
{
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

//	Matcher::matchBruteForce(*mAnchorImage, *mCurrentImage, matcherToAnchor);
	Matcher::matchOpticalFlow(*mAnchorImage, *mCurrentImage, matcherToAnchor);

	return true;
}
*/

const cv::Size optFlowWindowSize(15, 15);
const int maxLevel = 2;
const cv::TermCriteria optFlowStopCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT, 10, 0.03);


bool
VisualOdometry::runMatching (cv::Mat img, const ptime &timestamp, cv::Mat mask)
{
	mCurrentImage = BaseFrame::create(img, param.camera);
	if (frameCounter==0)
		mCurrentImage->setPose(Pose::Identity());

	if (voFeatureTracker.size()>0) {
		auto trackedFeatsPoints = voFeatureTracker.trackedFeaturesAtFrame(frameCounter);
		auto &_featureTrackIds = voFeatureTracker.getFeatureTrackIdsAtFrame(frameCounter);
		assert(trackedFeatsPoints.rows==_featureTrackIds.size());
		vector<FeatureTrackList::TrackId> featureTrackIds(_featureTrackIds.begin(), _featureTrackIds.end());
		uint nextFrameCounter = frameCounter+1;
		cv::Mat p1, p0r, status, errOf;

		cv::calcOpticalFlowPyrLK(mAnchorImage->getImage(), mCurrentImage->getImage(), trackedFeatsPoints, p1, status, errOf, optFlowWindowSize, maxLevel, optFlowStopCriteria);
		cv::calcOpticalFlowPyrLK(mCurrentImage->getImage(), mAnchorImage->getImage(), p1, p0r, status, errOf, optFlowWindowSize, maxLevel, optFlowStopCriteria);

		cv::Mat absDiff(cv::abs(trackedFeatsPoints-p0r));

		for (int r=0; r<trackedFeatsPoints.rows; ++r) {

			if (absDiff.at<float>(r,0)>=1 or absDiff.at<float>(r,1)>=1)
				continue;
			cv::Point2f pt(p1.row(r));

			// Add tracked point
			auto trackId = featureTrackIds[r];
			voFeatureTracker.addTrackedPoint(nextFrameCounter, trackId, pt);
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

		mCurrentImage->computeFeatures(featureDetector, fMask);
		// Add all features as new track
		for (int i=0; i<mCurrentImage->numOfKeyPoints(); ++i) {
			FeatureTrack ftNew(frameCounter, mCurrentImage->keypoint(i).pt);
			voFeatureTracker.add(ftNew);
		}
	}

	frameCounter+=1;
	drawFlow(img);
	mAnchorImage = mCurrentImage;
	return true;
}


bool
VisualOdometry::process(cv::Mat img, const ptime &timestamp, cv::Mat mask)
{
	bool bMatch = runMatching(img, timestamp, mask);
	frameCounter +=1;

	if (bMatch==false) return false;

	// Get transformation, and inliers
	TTransform motion = Matcher::calculateMovement(*mAnchorImage, *mCurrentImage, matcherToAnchor, matcherToAnchor);
	if (matcherToAnchor.size()<=10)
		return false;

	Pose pCurrent = mAnchorImage->pose() * motion;
	mCurrentImage->setPose(pCurrent);
	mVoTrack.push_back(PoseStamped(mCurrentImage->pose(), timestamp));

	map<uint, Vector3d> mapPoints;
	float parallax;
	TriangulateCV(*mAnchorImage, *mCurrentImage, matcherToAnchor, mapPoints, &parallax);
	for (auto &pt3dPr: mapPoints) {
		points3d->push_back(PointType(pt3dPr.second.x(), pt3dPr.second.y(), pt3dPr.second.z()));
	}
	cout << "Found " << mapPoints.size() << " points" << endl;

	return true;
}


void
VisualOdometry::drawFlow(cv::Mat canvas)
{
	_flowCanvas = canvas.clone();
}


TTransform
VisualOdometry::estimateMotion()
{

}


} /* namespace Vmml */
