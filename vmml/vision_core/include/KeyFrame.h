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

#include "utilities.h"
#include "BaseFrame.h"


namespace Vmml {


class VisionMap;


class KeyFrame : public BaseFrame
{
public:
	KeyFrame(const std::shared_ptr<VisionMap> _parent);
	virtual ~KeyFrame();

protected:
	kfid id;
	int cameraId;

	// Time at which the image was taken
	ptime frCreationTime;

	static kfid nextId;

	const std::shared_ptr<VisionMap> parent;
};

} /* namespace Vmml */

#endif /* VMML_CORE_KEYFRAME_H_ */
