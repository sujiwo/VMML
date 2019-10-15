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
#include "Pose.h"


namespace Vmml {


template<typename T>
class Grid
{
public:
	Grid(int width, int height, const T*(constFunc)()=NULL)
	{
		mGrid.resize(height);
		for (int y=0; y<height; y++) {
			mGrid[y].resize(width);
			for (int x=0; x<width; x++) {
				if (constFunc==NULL)
					mGrid[y][x] = new T*;
				else
					mGrid[y][x] = constFunc();
			}
		}
	}

	const T& operator() (const int x, const int y) const
	{ return mGrid[y][x]; }

	T& operator() (const int x, const int y)
	{ return mGrid[y][x]; }

protected:
	std::vector<std::vector<std::shared_ptr<T>>> mGrid;
};


class VisualOdometry {
public:
	struct Parameters {
		CameraPinholeParams camera;
		uint ransac_iters;
		double inlier_threshold,
		       motion_threshold;
		bool multiPass = false;
	};

	VisualOdometry(Parameters par);
	virtual ~VisualOdometry();

protected:
	Parameters param;
	Grid<std::vector<cv::KeyPoint>> featureGrid;
};

} /* namespace Vmml */

#endif /* VMML_CORE_VISUALODOMETRY_H_ */
