/*
 * RVizConnector.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include "RVizConnector.h"

namespace Vmml {
namespace Mapper {

RVizConnector::RVizConnector(int argc, char *argv[], const std::string &nodeName)
{
	ros::init(argc, argv, nodeName);
}

RVizConnector::~RVizConnector() {
	// TODO Auto-generated destructor stub
}

} /* namespace Mapper */
} /* namespace Vmml */
