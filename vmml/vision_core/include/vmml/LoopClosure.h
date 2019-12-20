/*
 * LoopClosure.h
 *
 *  Created on: Nov 1, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_LOOPCLOSURE_H_
#define VMML_CORE_LOOPCLOSURE_H_


#include "VisionMap.h"
#include "BaseFrame.h"


namespace Vmml {


class LoopClosure
{
public:
	LoopClosure(const VisionMap::Ptr &);
	virtual ~LoopClosure();

	// Tell loop closure that a new keyframe has been added
	void hasKeyFrameAdded(const kfid);

	bool detect(const BaseFrame &bf);

protected:
	VisionMap::Ptr mParent;

	kfid lastKfLoop;
};

} /* namespace Vmml */

#endif /* VMML_CORE_LOOPCLOSURE_H_ */
