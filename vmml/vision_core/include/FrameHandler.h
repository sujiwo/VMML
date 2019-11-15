/*
 * FrameHandler.h
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_FRAMEHANDLER_H_
#define VMML_CORE_FRAMEHANDLER_H_

#include <memory>
#include "VisionMap.h"
#include "BaseFrame.h"
#include "KeyFrame.h"
#include "MapBuilder.h"
#include "MapBuilderLidar.h"


namespace Vmml {


class FrameHandler
{
public:
	typedef std::shared_ptr<FrameHandler> Ptr;

	FrameHandler(const VisionMap::Ptr &parent_):
		parent(parent_)
	{}

	virtual ~FrameHandler()
	{}

	virtual void BaseFrameFunc(const BaseFrame::Ptr &fr);
	virtual void KeyFrameFunc(const KeyFrame::Ptr &keyf);
	virtual void MbFrameFunc(const MapBuilder::TmpFrame::Ptr &fr);
	virtual void MblFrameFunc(const MapBuilderLidar::LidarImageFrame::Ptr &fr);

protected:
	VisionMap::Ptr &parent;
};


}	// namespace Vmml

#endif /* VMML_CORE_FRAMEHANDLER_H_ */
