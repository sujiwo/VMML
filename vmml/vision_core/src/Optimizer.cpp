/*
 * Optimizer.cpp
 *
 *  Created on: Oct 29, 2019
 *      Author: sujiwo
 */

#include <random>
#include <algorithm>
#include "vmml/Optimizer.h"


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

const float thHuber2D = sqrt(5.99);
const float thHuber3D = sqrt(7.815);


class PoseDisturb
{
public:
	typedef std::uniform_real_distribution<double> UDistT;
	typedef std::shared_ptr<UDistT> UDistPtr;


	PoseDisturb(const Vector3d &means):
		meanShift(means),
		distributions(vector<UDistPtr>(3, nullptr))
	{
		if (meanShift.x()!=0.0) {
			distributions[0] = createUniformDistribution(meanShift.x());
		}
		if (meanShift.y()!=0.0) {
			distributions[1] = createUniformDistribution(meanShift.y());
		}
		if (meanShift.z()!=0.0) {
			distributions[2] = createUniformDistribution(meanShift.z());
		}
	}

	Pose disturb(const Pose &p)
	{
		Vector3d sh = getRandomShift();
		return p.shift(sh);
	}

	static UDistPtr createUniformDistribution(const double &m)
	{
		return UDistPtr(new UDistT(-m, m));
	}

protected:
	Vector3d meanShift;
	default_random_engine generator;
	vector<UDistPtr> distributions;

	Vector3d getRandomShift()
	{
		Vector3d sh = Vector3d::Zero();
		for (int i=0; i<3; ++i)
			if (distributions[i]!=nullptr) {
				UDistT &dist = *distributions[i];
				sh[i] = dist(generator);
			}

		return sh;
	}
};


void VertexCameraMono::set(KeyFrame::Ptr &_f)
{
	kf = _f;
	setEstimate(kf->toSE3Quat());
}


void VertexMapPoint::set(MapPoint::Ptr &_p)
{
	mp = _p;
	setEstimate(mp->getPosition());
}


void EdgeProjectMonocular::set(VertexCameraMono *frame, VertexMapPoint *point)
{
	auto myMap = frame->kf->parent();
	Vector2d obs = frame->kf->keypointv(point->getKeyPointId(frame));
	setMeasurement(obs);
	setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
	setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame));

	auto mKeypoint = frame->kf->keypoint(point->getKeyPointId(frame));
	Matrix2d edgeInfo = Matrix2d::Identity() * 1.2 * (mKeypoint.octave+1);
	setInformation(Matrix2d::Identity() * 1.2 * (mKeypoint.octave+1));
}


Vector3d EdgeProjectMonocular::transformWorldPointToFrame
(const Vector3d &pointInWorld)
const
{
	const auto *vKf = static_cast<const VertexCameraMono*>(vertex(1));
	auto est = vKf->estimate();
	return est.map(pointInWorld);
}


bool EdgeProjectMonocular::isDepthPositive() const
{
	const g2o::VertexSBAPointXYZ &point = *static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
	Vector3d ptByCam = transformWorldPointToFrame(point.estimate());
	return (ptByCam.z() >= 0.0);
}


G2O_REGISTER_TYPE(VERTEX_CAMERA_MONO, VertexCameraMono);
G2O_REGISTER_TYPE(VERTEX_MAP_POINT, VertexMapPoint);
G2O_REGISTER_TYPE(EDGE_PROJECT_MONOCULAR, EdgeProjectMonocular);


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


void
Optimizer::LocalBundleAdjustment(VisionMap &origMap, const kfid &targetKf)
{
	// Find connected keyframes from targetKf
	vector<kfid> neighbourKfs = origMap.getKeyFramesComeInto(targetKf);
	std::sort(neighbourKfs.begin(), neighbourKfs.end());
	neighbourKfs.push_back(targetKf);

	// Local MapPoints seen in Local KeyFrames
	set<mpid> relatedMps;
	for (auto &kfl: neighbourKfs) {
		for (auto &mpair: origMap.allMapPointsAtKeyFrame(kfl)) {
			relatedMps.insert(mpair.first);
		}
	}

	// Fixed KeyFrames: those that see local MapPoints but not included in connected keyframes
	set<kfid> fixedKfs;
	for (auto &mp: relatedMps) {
		auto curRelatedKf = origMap.getRelatedKeyFrames(mp);
		for (auto &kf: curRelatedKf) {
			if (std::find(neighbourKfs.begin(), neighbourKfs.end(), kf) != neighbourKfs.end())
				continue;
			fixedKfs.insert(kf);
		}
	}

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg >();
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	map<int, kfid> vertexKfMap;
	map<kfid, int> vertexKfMapInv;
	map<mpid, int> vertexMpMapInv;
	int vId = 1;

	// Local KeyFrame vertices
	int i = 0;
	for (auto &kf: neighbourKfs) {
		auto Kf = origMap.keyframe(kf);
		auto localKfVertex = new VertexCameraMono;
		localKfVertex->set(Kf);
		localKfVertex->setId(vId);
		localKfVertex->setFixed(vId==1);
		optimizer.addVertex(localKfVertex);
		vertexKfMap[vId] = kf;
		vertexKfMapInv[kf] = vId;
		++i;
		vId++;
	}

	// Fixed keyframe vertices
	i = 0;
	for (auto &kf: fixedKfs) {
		auto Kf = origMap.keyframe(kf);
		auto fixedKfVertex = new VertexCameraMono;
		fixedKfVertex->set(Kf);
		fixedKfVertex->setId(vId);
		fixedKfVertex->setFixed(true);
		optimizer.addVertex(fixedKfVertex);
		vertexKfMap[vId] = kf;
		vertexKfMapInv[kf] = vId;
		++i;
		vId++;
	}

	// How many edges do we need ?
	int numEdges = 0;
	for (auto &mp: relatedMps)
		numEdges += origMap.countRelatedKeyFrames(mp);

	// Camera Parameter
	auto Camera0 = origMap.getCameraParameter(0);
	auto* myCamera = new g2o::CameraParameters(Camera0.f(), Camera0.principalPoints(), 0);
	myCamera->setId(0);
	optimizer.addParameter(myCamera);

	const double thHuberDelta = sqrt(5.991);

	// MapPoint Vertices
//	vector<g2o::VertexSBAPointXYZ> relatedMpVertices(relatedMps.size());
	vector<EdgeProjectMonocular*> edgesMpKf(numEdges);
	i = 0;
	int j = 0;
	for (auto &mp: relatedMps) {
		auto *vertexMp = new VertexMapPoint;
		auto Mp = origMap.mappoint(mp);
		vertexMp->setFixed(false);
		vertexMp->set(Mp);
		vertexMp->setId(vId);
		vertexMp->setMarginalized(true);
		optimizer.addVertex(vertexMp);

		vertexMpMapInv[mp] = vId;

		// Create Edges
		for (auto &kf: origMap.getRelatedKeyFrames(mp)) {

			auto *edge = new EdgeProjectMonocular;
			edge->setParameterId(0, 0);
			edgesMpKf[j] = edge;

			auto vKf = dynamic_cast<VertexCameraMono*>(optimizer.vertex(vertexKfMapInv.at(kf)));
			edge->set(vKf, vertexMp);

			// Debugging purpose
			Vector3d
				transvec1 = edge->transformWorldPointToFrame(Mp->getPosition()),
				transvec2 = origMap.keyframe(kf)->transform(Mp->getPosition());

			auto *robustKernel = new g2o::RobustKernelHuber;
			edge->setRobustKernel(robustKernel);
			robustKernel->setDelta(thHuberDelta);

			optimizer.addEdge(edge);
			j++;
		}

		++i;
		vId++;
	}

	optimizer.initializeOptimization();
	optimizer.optimize(5);

	// Check inliers
	for (auto edge: edgesMpKf) {
		double c = edge->chi2();
		auto dp = edge->isDepthPositive();
		if (c > 5.991 or !dp) {
			edge->setLevel(1);
		}

		edge->setRobustKernel(nullptr);
	}

	// Re-optimize without outliers
	optimizer.initializeOptimization(0);
	optimizer.optimize(10);

	// Recover optimized data
	for (auto &kfl: neighbourKfs) {
		auto kfVtxId = vertexKfMapInv.at(kfl);
		auto vKf = static_cast<VertexCameraMono*> (optimizer.vertex(kfVtxId));
		vKf->updateToMap();
	}

	for (auto &mp: relatedMps) {
		auto mpVtxId = vertexMpMapInv.at(mp);
		auto vPoint = static_cast<VertexMapPoint*> (optimizer.vertex(mpVtxId));
		vPoint->updateToMap();
	}

	cout << "Local BA Done\n";
}


int
Optimizer::OptimizePose (const BaseFrame &frame, Pose &initPose, const VisionMap &vmap)
{

}


} /* namespace Vmml */
