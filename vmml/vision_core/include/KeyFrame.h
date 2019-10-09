/*
 * KeyFrame.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_KEYFRAME_H_
#define VMML_CORE_KEYFRAME_H_

#include <boost/serialization/serialization.hpp>
#include <limits>
#include <memory>

#include "utilities.h"
#include "BaseFrame.h"


namespace Vmml {


class VisionMap;


class KeyFrame : public BaseFrame
{
public:
	friend class VisionMap;

	KeyFrame(const std::shared_ptr<VisionMap> _parent);
	virtual ~KeyFrame();

	const std::shared_ptr<VisionMap> parent() const
	{ return mParent; }

	std::shared_ptr<VisionMap> parent()
	{ return mParent; }

	// XXX: Define more concrete constructor

	inline kfid getId() const
	{ return id; }

	static Ptr
	create(cv::Mat image, const std::shared_ptr<VisionMap>& mParent, int cameraNumber=0);

	static Ptr
	fromBaseFrame(const BaseFrame &frameSource, const std::shared_ptr<VisionMap>& mParent, int cameraNumber=0);

	typedef std::shared_ptr<KeyFrame> Ptr;

protected:
	kfid id;
	int cameraId;
	sourceId srcId;

	// Time at which the image was taken
	ptime frCreationTime;

	static kfid nextId;

	const std::shared_ptr<VisionMap> mParent;
};

} /* namespace Vmml */

#endif /* VMML_CORE_KEYFRAME_H_ */
