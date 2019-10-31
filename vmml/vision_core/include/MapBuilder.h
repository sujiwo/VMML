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
#include "VisionMap.h"
#include "Matcher.h"


namespace Vmml {


class MapBuilder
{
public:

	MapBuilder(const CameraPinholeParams &camera0);

	virtual bool feed(cv::Mat inputImage, const ptime &timestamp);

	virtual ~MapBuilder();

	std::shared_ptr<VisionMap>& getMap()
	{ return vMap; }

	typedef std::function<void(const KeyFrame &kf)> KeyFrameCreationCallback;
	inline void registerKeyFrameCallback (const KeyFrameCreationCallback &func)
	{ newKeyFrameCallback = func; }

	/*
	 * Temporary structure for map builder only
	 */
	struct TmpFrame : public BaseFrame
	{
	public:
		typedef std::shared_ptr<TmpFrame> Ptr;

		TmpFrame(cv::Mat img, std::shared_ptr<VisionMap> &_parent);
		static Ptr create(cv::Mat img, std::shared_ptr<VisionMap> &_parent);

		KeyFrame::Ptr toKeyFrame() const;

		bool initializeMatch(const KeyFrame::Ptr &key);

		bool track(const kfid &kf);

		bool isOkForKeyFrame() const;

		KeyFrame::Ptr parentKeyFrame;
		std::shared_ptr<VisionMap> parent;
		Matcher::PairList
			matchesToKeyFrame,			// all Inliers of matching to parent keyframe
			prevMapPointPairs,			// visible map points from parent keyframe
			candidatesMapPointPairs;	// candidates for new map points
		ptime timestamp;
	};

protected:

	std::shared_ptr<VisionMap> vMap;

	TmpFrame::Ptr currentWorkframe;

	bool initialize();

	bool track();

	bool hasInitialized = false;
	kfid lastAnchor = 0;
	CameraPinholeParams camera0;

	uint frameCounter = 0;

	KeyFrameCreationCallback newKeyFrameCallback;

	bool createInitialMap();

	void reset();
};

} /* namespace Vmml */

#endif /* VMML_CORE_MAPBUILDER_H_ */
