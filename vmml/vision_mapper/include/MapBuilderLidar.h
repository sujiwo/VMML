/*
 * MapBuilderLidar.h
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_MAPBUILDERLIDAR_H_
#define VMML_MAPPER_MAPBUILDERLIDAR_H_

#include "RandomAccessBag.h"
#include "MapBuilder.h"


namespace Vmml {
namespace Mapper {

class MapBuilderLidar : public Vmml::MapBuilder
{
public:
	MapBuilderLidar();
	virtual ~MapBuilderLidar();
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_MAPBUILDERLIDAR_H_ */
