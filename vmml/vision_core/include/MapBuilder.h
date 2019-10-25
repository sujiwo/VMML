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


namespace Vmml {

class MapBuilder
{
public:

	MapBuilder(const CameraPinholeParams &camera0);

	virtual bool feed(cv::Mat inputImage);

	virtual ~MapBuilder();

	std::shared_ptr<VisionMap>& getMap()
	{ return vMap; }

protected:

	std::shared_ptr<VisionMap> vMap;

	bool initialize(BaseFrame::Ptr &f);

	bool track(BaseFrame::Ptr &fr);

	bool hasInitialized = false;
	kfid lastAnchor = 0;
	CameraPinholeParams camera0;

	bool requireNewKeyFrame(const BaseFrame &f);
};

} /* namespace Vmml */

#endif /* VMML_CORE_MAPBUILDER_H_ */
