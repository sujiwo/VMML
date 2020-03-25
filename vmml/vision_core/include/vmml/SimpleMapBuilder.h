/*
 * MapBuilder.h
 *
 *  Created on: Oct 8, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_MAPBUILDER_H_
#define VMML_CORE_MAPBUILDER_H_

/*
 * Basic Visual Map Builder that does not require external positioning.
 * Supports only one camera
 */

#include <memory>
#include <limits>
#include <functional>
#include "utilities.h"
#include "CameraPinholeParams.h"
#include "ImageDatabase.h"
#include "KeyFrame.h"
#include "VisionMap.h"
#include "Matcher.h"
#include "LoopClosure.h"


namespace Vmml {


class SimpleMapBuilder
{
public:
	struct Parameters {
		CameraPinholeParams camera;
		int numOfFeatures;
		// maximum distance (in #of keyframes) since last local BA
		int maxDistanceLocalBA=10;
		// More values to come
	};

	SimpleMapBuilder(const Parameters &pars);

	virtual bool process(const cv::Mat &inputImage, const ptime &timestamp, const cv::Mat &mask=cv::Mat());

	virtual ~SimpleMapBuilder();

	std::shared_ptr<VisionMap>& getMap()
	{ return vMap; }

	const std::shared_ptr<VisionMap>& getMap() const
	{ return vMap; }

	/*
	 * Temporary structure for map builder
	 */
	struct TmpFrame : public BaseFrame
	{
	public:
		typedef std::shared_ptr<TmpFrame> Ptr;

		TmpFrame(cv::Mat img, std::shared_ptr<VisionMap> &_parent, const cv::Mat &mask);
		static Ptr create(cv::Mat img, std::shared_ptr<VisionMap> &_parent, const cv::Mat &mask);

		KeyFrame::Ptr toKeyFrame() const;

		bool initializeMatch(const KeyFrame::Ptr &key, std::vector<Eigen::Vector3d> &trPoints);

		bool track(const kfid &kf);

		cv::Mat visualize() const;

		bool isOkForKeyFrame() const;

		KeyFrame::Ptr parentKeyFrame=nullptr;
		std::shared_ptr<VisionMap> parent;

		Matcher::PairList
			matchesToKeyFrame,			// all Inliers of matching to parent keyframe
			prevMapPointPairs,			// visible map points from parent keyframe
			candidatesMapPointPairs;	// candidates for new map points
		ptime timestamp;

		uint frameId=0;
	};

	typedef std::function<void(const TmpFrame &)> FrameCreationCallback;
	inline void registerFrameCallback (const FrameCreationCallback &func)
	{ newFrameCallback = func; }

	inline const TmpFrame::Ptr& getCurrentFrame() const
	{ return currentWorkframe; }

	/*
	 * Call this when image stream has ended (or should mapping be finished)
	 */
	void end();

	const Trajectory& getTrajectory() const
	{ return frameTrajectory; }

protected:

	Parameters smParameters;

	std::shared_ptr<VisionMap> vMap;

	TmpFrame::Ptr currentWorkframe;

	bool initialize();

	bool track();

	bool hasInitialized = false;
	kfid lastAnchor = 0;

	uint frameCounter = 0;

	FrameCreationCallback newFrameCallback;

	bool createInitialMap(const std::vector<Eigen::Vector3d> &initialTriangulatedPoints);

	bool createNewKeyFrame();

	void trackMapPoints(const kfid k1, const kfid k2);

	void reset();

	inline void callFrameFunction() const
	{
		if (newFrameCallback)
			newFrameCallback(*currentWorkframe);
	}

	Trajectory frameTrajectory;
};

} /* namespace Vmml */

#endif /* VMML_CORE_MAPBUILDER_H_ */
