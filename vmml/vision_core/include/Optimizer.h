/*
 * Optimizer.h
 *
 *  Created on: Oct 29, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_OPTIMIZER_H_
#define VMML_CORE_OPTIMIZER_H_

#include "KeyFrame.h"
#include "VisionMap.h"

namespace Vmml {


typedef uint64 oid;


class Optimizer {
public:

	static void
	BundleAdjustment(VisionMap &myMap, const int bIteration);

	static int
	OptimizePose (const BaseFrame &frame, Pose &initPose, const VisionMap &vmap);
};

} /* namespace Vmml */

#endif /* VMML_CORE_OPTIMIZER_H_ */
