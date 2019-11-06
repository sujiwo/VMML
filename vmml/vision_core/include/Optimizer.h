/*
 * Optimizer.h
 *
 *  Created on: Oct 29, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_OPTIMIZER_H_
#define VMML_CORE_OPTIMIZER_H_

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/factory.h"

#include "KeyFrame.h"
#include "VisionMap.h"

namespace Vmml {


typedef uint64 oid;


class G2O_TYPES_SBA_API VertexCameraMono : public g2o::VertexSE3Expmap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexCameraMono() : g2o::VertexSE3Expmap() {}

	void set(KeyFrame::Ptr _f);

	void updateToMap()
	{ return kf->setPose(estimate()); }

	kfid getIdFromMap () const
	{ return kf->getId(); }

	KeyFrame::Ptr kf = nullptr;
};


class G2O_TYPES_SBA_API VertexMapPoint : public g2o::VertexSBAPointXYZ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexMapPoint() : g2o::VertexSBAPointXYZ() {}

	void set(MapPoint::Ptr _mp);

	void updateToMap()
	{ return mp->setPosition(estimate()); }

	mpid getIdFromMap() const
	{ return mp->getId(); }

	kpid getKeyPointId(const KeyFrame &k) const
	{
		return k.parent()->getKeyPointId(k.getId(), getIdFromMap());
	}

	kpid getKeyPointId(const VertexCameraMono *k) const
	{
		return getKeyPointId(*k->kf);
	}

	MapPoint::Ptr mp = nullptr;
};


class G2O_TYPES_SBA_API EdgeProjectMonocular : public g2o::EdgeProjectXYZ2UV
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeProjectMonocular() : g2o::EdgeProjectXYZ2UV() {}

	/*
	 * Perform multiple jobs: set estimation and information matrix
	 */
	void set(VertexCameraMono *_f, VertexMapPoint *_p);
	Vector3d transformWorldPointToFrame(const Vector3d &pointInWorld) const;
	bool isDepthPositive() const;
};


class G2O_TYPES_SBA_API EdgeFrameMovement : public g2o::EdgeSE3Expmap
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeFrameMovement() : g2o::EdgeSE3Expmap() {}
	void set(VertexCameraMono &f1, VertexCameraMono &f2);
};


class Optimizer {
public:

	static void
	BundleAdjustment(VisionMap &myMap, const int bIteration);

	static void
	LocalBundleAdjustment(VisionMap &myMap, const kfid &targetKf);

	static int
	OptimizePose (const BaseFrame &frame, Pose &initPose, const VisionMap &vmap);
};

} /* namespace Vmml */

#endif /* VMML_CORE_OPTIMIZER_H_ */
