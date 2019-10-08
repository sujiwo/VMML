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
#include "utilities.h"
#include "VisionMap.h"
#include "RandomAccessBag.h"


namespace Vmml {

class MapBuilder
{
public:

	MapBuilder(const CameraPinholeParams &camera0, float zoom);

	void build(const RandomAccessBag &imageBag,
		sourceId start=0,
		sourceId stop=std::numeric_limits<sourceId>::max());

	virtual ~MapBuilder();

	std::shared_ptr<VisionMap>& getMap()
	{ return vMap; }

protected:

	std::shared_ptr<VisionMap> vMap;
	float zoomRatio;

	void initialize();
	void feed();

	bool hasInitialized = false;

	KeyFrame::Ptr createKeyFrame(const RandomAccessBag &imageBag, sourceId n) const;
};

} /* namespace Vmml */

#endif /* VMML_CORE_MAPBUILDER_H_ */
