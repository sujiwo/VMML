/*
 * TrajectoryGNSS.h
 *
 *  Created on: Nov 26, 2019
 *      Author: sujiwo
 */

#include <string>
#include "Trajectory.h"
#include "rosbag/bag.h"


#ifndef VMML_CORE_TRAJECTORYGNSS_H_
#define VMML_CORE_TRAJECTORYGNSS_H_

namespace Vmml {

class TrajectoryGNSS : public Trajectory
{
public:
	static TrajectoryGNSS
	fromRosBag(rosbag::Bag &bag, const std::string &topicName, TTransform worldToMap, int plane_number=7);

protected:
};

} /* namespace Vmml */

#endif /* VMML_CORE_TRAJECTORYGNSS_H_ */
