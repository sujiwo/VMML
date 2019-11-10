/*
 * VisualOdometry.h
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_VISUALODOMETRY_H_
#define VMML_CORE_VISUALODOMETRY_H_

#include <memory>
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
		       motion_threshold;
		bool multiPass = false;

		int bucket_width = 10,
			bucket_height = 10;
	};

	VisualOdometry(Parameters par);
	virtual ~VisualOdometry();

	bool process (cv::Mat img);

//	bool process (const BaseFrame &newFrame, const Matcher::PairList &matchList);

protected:
	BaseFrame::Ptr mCurrentImage;
	Parameters param;
	Grid<std::vector<cv::KeyPoint>> featureGrid;

	void updateMotion();
};

} /* namespace Vmml */

#endif /* VMML_CORE_VISUALODOMETRY_H_ */
