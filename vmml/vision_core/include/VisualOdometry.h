/*
 * VisualOdometry.h
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_VISUALODOMETRY_H_
#define VMML_CORE_VISUALODOMETRY_H_

#include <memory>
#include <opencv2/features2d.hpp>
#include "utilities.h"
#include "CameraPinholeParams.h"
#include "BaseFrame.h"
#include "Matcher.h"
#include "Pose.h"


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

	bool process (cv::Mat img);

//	bool process (const BaseFrame &newFrame, const Matcher::PairList &matchList);

protected:

	Parameters param;
	cv::Ptr<cv::FeatureDetector> featureDetector;
	Grid<std::vector<cv::KeyPoint>> featureGrid;

	BaseFrame::Ptr
		mAnchorImage=nullptr,
		mCurrentImage=nullptr;
	Matcher::PairList matcherToAnchor;

	TTransform estimateMotion();
};

} /* namespace Vmml */

#endif /* VMML_CORE_VISUALODOMETRY_H_ */
