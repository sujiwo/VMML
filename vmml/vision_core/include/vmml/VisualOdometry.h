/*
 * VisualOdometry.h
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_VISUALODOMETRY_H_
#define VMML_CORE_VISUALODOMETRY_H_

#include <vector>
#include <set>
#include <memory>
#include <opencv2/features2d.hpp>
#include "utilities.h"
#include "CameraPinholeParams.h"
#include "BaseFrame.h"
#include "Matcher.h"
#include "Trajectory.h"
#include "MapBuilderLidar.h"
#include "FeatureTrack.h"


namespace Vmml {


class VisualOdometry {
public:

	struct Parameters {
		CameraPinholeParams camera;
		uint ransac_iters;

		double inlier_threshold,
		       motion_threshold=100.0;
		bool multiPass = false;

		double cameraHeight = 1.0;

		int bucket_width = 10,
			bucket_height = 10;
	};

	VisualOdometry(Parameters par);
	virtual ~VisualOdometry();

	bool runMatching (cv::Mat img, const ptime &timestamp, cv::Mat mask=cv::Mat());

	bool process (cv::Mat img, const ptime &timestamp, cv::Mat mask=cv::Mat(), bool matchOnly=false);

	inline const Trajectory& getTrajectory() const
	{ return mVoTrack; }

	inline const LocalLidarMapper::CloudType::Ptr getPoints() const
	{ return points3d; }

	inline uint getInlier() const
	{ return voMatcherToAnchor.size(); }

	const BaseFrame::Ptr getAnchorFrame() const
	{ return mAnchorImage; }

	const BaseFrame::Ptr getCurrentFrame() const
	{ return mCurrentImage; }

	const Matcher::PairList& getLastMatch() const
	{ return voMatcherToAnchor; }

	// XXX: Temporary
	cv::Mat _flowCanvas;
	void drawFlow(cv::Mat canvas);

//	bool process (const BaseFrame &newFrame, const Matcher::PairList &matchList);

protected:

	Parameters param;
	cv::Ptr<cv::ORB> featureDetector;
	Grid<std::vector<cv::KeyPoint>> featureGrid;
	Trajectory mVoTrack;
	LocalLidarMapper::CloudType::Ptr points3d;

	BaseFrame::Ptr
		mAnchorImage=nullptr,
		mCurrentImage=nullptr;

	// Matching results
	Matcher::PairList
		flowMatcherToAnchor,
		voMatcherToAnchor;

	TTransform estimateMotion();

	const uint maxNumberFeatures = 6000;
	const uint featureDetectionInterval = 5;

	FeatureTrackList voFeatureTracker;

	uint frameCounter = 0;
};

} /* namespace Vmml */

#endif /* VMML_CORE_VISUALODOMETRY_H_ */
