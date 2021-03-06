/*
 * VisionMap.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#include <memory>
#include <algorithm>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>

#include "vmml/KeyFrame.h"
#include "vmml/MapPoint.h"
#include "vmml/VisionMap.h"
#include "vmml/MapObjectSerialization.h"

#define MAX_ORB_POINTS_IN_FRAME 9000

using namespace std;
using namespace Eigen;


namespace Vmml {


VisionMap::VisionMap() :

	keyframeInvIdx_mtx(new mutex),
	mappointInvIdx_mtx(new mutex)

{
	// Feature detector
	cv::Ptr<cv::ORB> orbf = cv::ORB::create(
		MAX_ORB_POINTS_IN_FRAME,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);
	featureDetector = orbf;

	// Fill scale factors
	int level = orbf->getNLevels();
	mScaleFactors.resize(level);
	mLevelSigma2.resize(level);
	mScaleFactors[0] = 1.0;
	mLevelSigma2[0] = 1.0;
	for (int i=1; i<level; ++i) {
		mScaleFactors[i] = mScaleFactors[i-1] * orbf->getScaleFactor();
		mLevelSigma2[i] = mScaleFactors[i] * mScaleFactors[i];
	}

	// Descriptor matcher
	descriptorMatcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
}


VisionMap::~VisionMap() {
	// TODO Auto-generated destructor stub
}


void
VisionMap::reset()
{
	keyframeInvIdx.clear();
	mappointInvIdx.clear();
	pointAppearances.clear();
	framePoints.clear();
	framePointsInv.clear();
}


int
VisionMap::addCameraParameter (const CameraPinholeParams &vscamIntr)
{
	cameraList.push_back(vscamIntr);
	return cameraList.size()-1;
}


bool
VisionMap::addKeyFrame(KeyFrame::Ptr frame)
{
	auto nId = frame->getId();

	keyframeInvIdx_mtx->lock();
	keyframeInvIdx.insert(make_pair(nId, frame));
	keyframeInvIdx_mtx->unlock();

	/*
	 * image database part
	 * XXX: Temporarily disabled
	 */
	if (imageDb.numImages()==0) {
		imageDb.addImage(nId, frame->allKeypoints(), frame->allDescriptors());
	}
	else {
		imageDb.addImage2(nId, frame->allKeypoints(), frame->allDescriptors());
	}

	// This keyframe has no map points yet
	framePoints[nId] = map<mpid,kpid>();
	framePointsInv[nId] = map<kpid,mpid>();

	// Graph
	auto vtId = boost::add_vertex(covisibility);
	kfVtxMap[nId] = vtId;
	kfVtxInvMap[vtId] = nId;

	return true;
}


bool
VisionMap::addMapPoint(MapPoint::Ptr mp)
{
	mpid nId = mp->getId();
	mappointInvIdx.insert(make_pair (nId, mp));

	return true;
}


void
VisionMap::addMapPointVisibility(const mpid &mp, const kfid &kf, const kpid &kp)
{
	// Add relationship between map point and keyframe, update graph
	framePoints[kf].insert(make_pair(mp, kp));
	framePointsInv[kf].insert(make_pair(kp, mp));
	pointAppearances[mp].insert(kf);
}


map<mpid,kpid>
VisionMap::allMapPointsAtKeyFrame(const kfid f)
const
{
	if (framePoints.size()==0)
		return map<mpid,kpid>();

	return framePoints.at(f);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
VisionMap::dumpPointCloudFromMapPoints ()
const
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr mapPtCl
		(new pcl::PointCloud<pcl::PointXYZ>(mappointInvIdx.size(), 1));

	uint64 i = 0;
	for (auto &mpIdx: mappointInvIdx) {
		auto &mp = mpIdx.second;
		mapPtCl->at(i).x = mp->X();
		mapPtCl->at(i).y = mp->Y();
		mapPtCl->at(i).z = mp->Z();
		i++;
	}

	return mapPtCl;
}


vector<kfid>
VisionMap::allKeyFrames () const
{
	vector<kfid> kfIdList;
	for (auto &key: keyframeInvIdx) {
		kfIdList.push_back(key.first);
	}
	return kfIdList;
}


vector<mpid>
VisionMap::allMapPoints () const
{
	vector<mpid> mpIdList;
	for (auto &key: mappointInvIdx) {
		mpIdList.push_back(key.first);
	}
	return mpIdList;
}


std::vector<mpid>
VisionMap::getVisibleMapPoints (const kfid &kf) const
{
	vector<mpid> rv;
	map<mpid, kpid> mpList;
	try {
		mpList = framePoints.at(kf);
	} catch (std::out_of_range &e) {
		return rv;
	}
	for (auto &mp: framePoints.at(kf))
		rv.push_back(mp.first);
	return rv;
}


bool
VisionMap::removeMapPoint (const mpid &i)
{
	assert(mappointInvIdx.find(i) != mappointInvIdx.end());

	mappointInvIdx.erase(i);
	set<kfid> ptAppears = pointAppearances[i];
	for (auto k: ptAppears) {
		const kpid kp = framePoints[k].at(i);
		framePoints[k].erase(i);
//		framePointsInv[k].erase(kp);
	}

	pointAppearances.erase(i);

	// Modify Graph
	for (auto k: ptAppears) {
		updateCovisibilityGraph(k);
	}

	return true;
}


bool
VisionMap::removeMapPointsBatch (const vector<mpid> &mplist)
{
	set<kfid> kfModified;

	for (auto &pt: mplist) {
		mappointInvIdx.erase(pt);
		set<kfid> kfAppears = pointAppearances[pt];
		for (auto kf: kfAppears) {
			kfModified.insert(kf);
			const kpid kp = framePoints[kf].at(pt);
			framePoints[kf].erase(pt);
		}

		pointAppearances.erase(pt);
	}

	for (auto kf: kfModified) {
		updateCovisibilityGraph(kf);
	}

	return true;
}


void
VisionMap::updateCovisibilityGraph(const kfid k)
{
	map<kfid,int> kfCounter;

	for (auto mp_ptr: framePoints.at(k)) {
		mpid pId = mp_ptr.first;
		for (auto kr: pointAppearances.at(pId)) {
			if (kr==k)
				continue;
			kfCounter[kr]++;
		}
	}

	if (kfCounter.empty())
		return;

//	Should we clear the vertex?
//	boost::clear_vertex(kfVtxMap[k], covisibility);
	for (auto kfctr: kfCounter) {
		covisibilityGraphMtx.lock();
		// XXX: Do NOT put KfID directly to graph; use vertex descriptor instead
		boost::add_edge(kfVtxMap[k], kfVtxMap[kfctr.first], kfctr.second, covisibility);
		covisibilityGraphMtx.unlock();
	}
}


void
VisionMap::updateMapPointDescriptor(const mpid mp)
{
	auto mPoint = mappoint(mp);
	vector<KeyMapPoint> vKf;
	for(auto &kfId: pointAppearances.at(mp)) {
		auto currentKeyframe = keyframe(kfId);
		kpid kp = framePoints.at(kfId).at(mp);
		KeyMapPoint kmp = {*currentKeyframe, kp};
		vKf.push_back(kmp);
	}

	if (vKf.size() >= 3)
		mPoint->createDescriptor(vKf);
}


uint
VisionMap::getTrackedMapPointsAt(const kfid &kf, const uint minNumberOfObservation) const
{
	uint nPoints=0;
	auto Kf = keyframe(kf);

	for (auto mpPair: framePoints.at(kf)) {
		auto mp = mpPair.first;
		if (getNumberObservations(mp)>=minNumberOfObservation) {
			nPoints++;
		}
	}

	return nPoints;
}


std::vector<kfid>
VisionMap::getKeyFramesComeInto (const kfid kTarget) const
{
	set<kfid> kfListSrc;

	auto kvtx = kfVtxMap.at(kTarget);
	auto k_in_edges = boost::in_edges(kvtx, covisibility);
	for (auto vp=k_in_edges.first; vp!=k_in_edges.second; ++vp) {
		auto v = boost::source(*vp, covisibility);
		int w = boost::get(boost::edge_weight_t(), covisibility, *vp);
		kfListSrc.insert(kfVtxInvMap.at(v));
	}

	return vector<kfid>(kfListSrc.begin(), kfListSrc.end());
}


std::vector<KeyFrame::Ptr>
VisionMap::getSortedKeyframes() const
{
	vector<KeyFrame::Ptr> kfList(keyframeInvIdx.size());

	int i=0;
	for (auto &p: keyframeInvIdx) {
		kfList[i] = p.second;
		i++;
	}

	sort(kfList.begin(), kfList.end(), [&](const KeyFrame::Ptr &k1, const KeyFrame::Ptr &k2){
		return k1->frCreationTime < k2->frCreationTime;
	});
	return kfList;
}

Trajectory
VisionMap::dumpCameraTrajectory () const
{
	Trajectory camTrack;

	auto kfList = getSortedKeyframes();
	for (auto &cameraKey: kfList) {
		camTrack.push_back(PoseStamped(cameraKey->pose(), cameraKey->frCreationTime));
	}

	return camTrack;
}


bool
VisionMap::save (const std::string &path) const
{
	fstream mapFileFd;
	mapFileFd.open(path, fstream::out | fstream::trunc);
	if (!mapFileFd.is_open())
		throw runtime_error("Unable to create map file");
	boost::archive::binary_oarchive mapStore(mapFileFd);

	uint32_t
		numKf = keyframeInvIdx.size(),
		numMp = mappointInvIdx.size();
	mapStore << numKf;
	mapStore << numMp;

	mapStore << pointAppearances;
	mapStore << framePoints;
	mapStore << framePointsInv;

	mapStore << cameraList;

	mapStore << covisibility;
	mapStore << kfVtxMap;
	mapStore << kfVtxInvMap;

	mapStore << keyValueInfo;

	for (auto &kfPtr: keyframeInvIdx) {
		auto &kf = kfPtr.second;
		mapStore << *kf;
	}

	// Mappoints
	for (auto &mpPtr: mappointInvIdx) {
		auto &mp = mpPtr.second;
		mapStore << *mp;
	}

	// Image Database
	mapStore << imageDb;

	mapFileFd.close();
	return true;
}


bool
VisionMap::load (const std::string &path)
{
	cout << "Loading map `" << path << "`... ";
	fstream mapFileFd;
	mapFileFd.open(path.c_str(), fstream::in);
	if (!mapFileFd.is_open())
		throw runtime_error(string("Unable to open map file: ") + path);

	boost::archive::binary_iarchive mapStore (mapFileFd);

	uint32_t numOfKf, numOfMp;
	mapStore >> numOfKf;
	mapStore >> numOfMp;

	mapStore >> pointAppearances;
	mapStore >> framePoints;
	mapStore >> framePointsInv;

	mapStore >> cameraList;

	mapStore >> covisibility;
	mapStore >> kfVtxMap;
	mapStore >> kfVtxInvMap;

	mapStore >> keyValueInfo;

	auto myself = shared_from_this();
	for (int i=0; i<numOfKf; ++i) {
		KeyFrame::Ptr mKf = make_shared<KeyFrame>(myself);
		mapStore >> *mKf;
		mKf->cameraParam = cameraList[mKf->cameraId];
		keyframeInvIdx.insert(make_pair(mKf->id, mKf));
	}

	for (int i=0; i<numOfMp; ++i) {
		MapPoint::Ptr mPt = make_shared<MapPoint>();
		mapStore >> *mPt;
		mappointInvIdx.insert(make_pair(mPt->getId(), mPt));
	}

	mapStore >> imageDb;

	mapFileFd.close();
	cout << "Done\n";
	return true;
}


std::vector<kfid>
VisionMap::findCandidates (BaseFrame &queryFrame) const
{
	queryFrame.computeFeatures(featureDetector);

	vector<vector<cv::DMatch>> featureMatches;
	imageDb.searchDescriptors(queryFrame.allDescriptors(), featureMatches, 2, 64);
	// Filter matches according to ratio test
	vector<cv::DMatch> realMatches;
	for (uint m=0; m<featureMatches.size(); m++) {
		if (featureMatches[m][0].distance < featureMatches[m][1].distance * 0.8)
			realMatches.push_back(featureMatches[m][0]);
	}

	vector<ImageMatch> imageMatches;
	imageDb.searchImages(queryFrame.allDescriptors(), realMatches, imageMatches);

	vector<kfid> keyframeMatches;
	for (auto &im: imageMatches) {
		keyframeMatches.push_back(im.image_id);
	}
	return keyframeMatches;
}


vector<kfid>
VisionMap::getOrderedRelatedKeyFramesFrom (const kfid kx, int howMany) const
{
	vector<pair<KeyFrameGraph::vertex_descriptor,int>> covisk;
	map<KeyFrameGraph::vertex_descriptor,int> coviskWithWeight;

	auto vtx = kfVtxMap.at(kx);
	auto kfl = boost::out_edges(vtx, covisibility);
	for (auto p=kfl.first; p!=kfl.second; ++p) {
		auto k = boost::target(*p, covisibility);
		int w = boost::get(boost::edge_weight_t(), covisibility, *p);
		coviskWithWeight[k]=w;
	}
	for (auto &pr: coviskWithWeight)
		covisk.push_back(make_pair(pr.first, pr.second));

	sort(covisk.begin(), covisk.end(),
		[](const pair<kfid,int> &u1, const pair<kfid,int> &u2) -> bool
		{ return u1.second > u2.second;}
	);

	vector<kfid> sortedKfs(covisk.size());
	for (int i=0; i<covisk.size(); i++) {
		sortedKfs[i] = kfVtxInvMap.at(covisk.at(i).first);
	}

	if (howMany<0 or sortedKfs.size()<howMany)
		return sortedKfs;
	else
		return vector<kfid> (sortedKfs.begin(), sortedKfs.begin()+howMany);
}


} /* namespace Vmml */
