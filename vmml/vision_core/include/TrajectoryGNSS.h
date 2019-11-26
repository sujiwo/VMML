/*
 * TrajectoryGNSS.h
 *
 *  Created on: Nov 26, 2019
 *      Author: sujiwo
 */

#include "Trajectory.h"
#include "rosbag/bag.h"


#ifndef VMML_CORE_TRAJECTORYGNSS_H_
#define VMML_CORE_TRAJECTORYGNSS_H_

namespace Vmml {

class TrajectoryGNSS : public Trajectory
{
public:
	TrajectoryGNSS();
	virtual ~TrajectoryGNSS();

	static TrajectoryGNSS
	fromRosBag(rosbag::Bag &bag);



protected:
};

} /* namespace Vmml */

#endif /* VMML_CORE_TRAJECTORYGNSS_H_ */
