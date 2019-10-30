/*
 * Optimizer.cpp
 *
 *  Created on: Oct 29, 2019
 *      Author: sujiwo
 */

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/factory.h"

#include "Optimizer.h"


using namespace Eigen;
using namespace std;


template<typename LinearSolverT, typename BlockSolverT, typename OptimizationAlgorithmT>
OptimizationAlgorithmT*
createSolverAlgorithm()
{
	typename BlockSolverT::LinearSolverType* ls = new LinearSolverT();
	auto lsptr = std::unique_ptr<typename BlockSolverT::LinearSolverType>(ls);
	auto solver = std::unique_ptr<g2o::Solver>(new BlockSolverT(std::move(lsptr)));
	return new OptimizationAlgorithmT(std::move(solver));
}


namespace Vmml {


g2o::SE3Quat toSE3Quat (const KeyFrame &kf)
{
	Matrix4d extMat = kf.externalParamMatrix4();
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


g2o::SE3Quat toSE3Quat (const BaseFrame &frame)
{
	Matrix4d extMat = frame.externalParamMatrix4();
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


g2o::SE3Quat toSE3Quat (const Pose &spose)
{
	Matrix4d extMat = BaseFrame::createExternalParamMatrix4(spose);
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


void fromSE3Quat (const g2o::SE3Quat &pose, KeyFrame &kf)
{
	kf.orientation() = pose.rotation().inverse();
	kf.position() = -(kf.orientation() * pose.translation());
}


void fromSE3Quat (const g2o::SE3Quat &pose, BaseFrame &frb)
{
	auto Q = pose.rotation().inverse();
	auto P = -(Q * pose.translation());
	frb.setPose(P, Q);
}


void fromSE3Quat (const g2o::SE3Quat &pose, Pose &target)
{
	auto Q = pose.rotation().inverse();
	auto P = -(Q * pose.translation());
	target = Pose::from_Pos_Quat(P, Q);
}


#include <cstdio>
void debugVector (const VectorXd &v)
{
	if (v.size()==2)
		printf ("(%f, %f)\n", v.x(), v.y());
	else if (v.size()==3)
		printf ("(%f, %f, %f)\n", v.x(), v.y(), v.z());
}


void
Optimizer::BundleAdjustment(VisionMap &orgMap, const int baIteration)
{
	vector<kfid> keyframeList = orgMap.allKeyFrames();
	vector<mpid> mappointList = orgMap.allMapPoints();

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
/*
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
*/
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg>();
	optimizer.setAlgorithm(solver);

	// XXX: This routine does not support multiple camera
	const CameraPinholeParams cp = orgMap.getCameraParameter(0);
	g2o::CameraParameters *camParams =
		new g2o::CameraParameters(cp.fx, Vector2d(cp.cx,cp.cy), 0);
	camParams->setId(0);
	optimizer.addParameter(camParams);

	map<oid, kfid> vertexKfMap;
	map<kfid, g2o::VertexSE3Expmap*> vertexKfMapInv;
	map<oid, mpid> vertexMpMap;
	map<mpid, g2o::VertexSBAPointXYZ*> vertexMpMapInv;
	oid vId = 1;

	for (kfid &kId: keyframeList) {

		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		auto kf = orgMap.keyframe(kId);
		vKf->setEstimate (toSE3Quat(*kf));
		vKf->setId(vId);
		vKf->setFixed(kId<2);
		optimizer.addVertex(vKf);
		vertexKfMap.insert(pair<oid, kfid> (vId, kId));
		vertexKfMapInv[kId] = vKf;
		vId ++;

		for (auto &ptr: orgMap.allMapPointsAtKeyFrame(kId)) {

			const auto mp = orgMap.mappoint(ptr.first);
			const cv::KeyPoint p2K = kf->keypoint(ptr.second);

			g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();

			// What effect if we run this line ?
			vMp->setFixed(false);

			vMp->setEstimate(mp->getPosition());
			vMp->setMarginalized(true);
			vMp->setId(vId);
			optimizer.addVertex(vMp);
			vertexMpMap.insert(pair<oid,mpid> (vId, mp->getId()));
			vertexMpMapInv[mp->getId()] = vMp;
			vId++;

			// Edges
			g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
			edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vMp));
			edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vKf));
			edge->setMeasurement(Vector2d(p2K.pt.x, p2K.pt.y));
			Matrix2d uncertainty = Matrix2d::Identity() * (1.2*(p2K.octave+1));
			edge->setInformation(uncertainty);
			edge->setParameterId(0, 0);

			// XXX: Should we add a robust kernel for the edge here, ie. like pose optimization below ?

			optimizer.addEdge(edge);
		}
	}

	optimizer.initializeOptimization();
	// XXX: Determine number of iterations
	optimizer.optimize(baIteration);

	// Recovery of optimized data
	// KeyFrames
	for (auto &kVpt: vertexKfMap) {

		oid vId = kVpt.first;
		kfid kId = kVpt.second;
		auto kf = orgMap.keyframe(kId);
		g2o::VertexSE3Expmap *vKfSE3 = static_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(vId));

		g2o::SE3Quat kfPoseSE3 = vKfSE3->estimate();
		fromSE3Quat(kfPoseSE3, *kf);
	}

	// MapPoints
	for (auto &mVpt: vertexMpMap) {
		oid vId = mVpt.first;
		mpid mid = mVpt.second;
		auto mp = orgMap.mappoint(mid);
		g2o::VertexSBAPointXYZ *vMp = static_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(vId));
		mp->setPosition(vMp->estimate());
	}
}


int
Optimizer::OptimizePose (const BaseFrame &frame, Pose &initPose, const VisionMap &vmap)
{

}


} /* namespace Vmml */
