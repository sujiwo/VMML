/*
 * MapBuilder.cpp
 *
 *  Created on: Oct 8, 2019
 *      Author: sujiwo
 */

#include "MapBuilder.h"
#include "BaseFrame.h"
#include "Matcher.h"
#include "Triangulation.h"


using namespace std;
using namespace Eigen;


namespace Vmml {


MapBuilder::TmpFrame::
	TmpFrame(cv::Mat img, std::shared_ptr<VisionMap> &_parent) :
		BaseFrame(img, _parent->getCameraParameter(0), Pose::Identity()),
		parent(_parent)
{
	computeFeatures(parent->getFeatureDetector());
}


MapBuilder::TmpFrame::Ptr
MapBuilder::TmpFrame::create(cv::Mat img, shared_ptr<VisionMap> &_parent)
{ return Ptr(new TmpFrame(img, _parent)); }


bool
MapBuilder::TmpFrame::initializeMatch(const KeyFrame::Ptr &kf)
{
	parentKeyFrame = kf;

	int Ns1 = Matcher::matchBruteForce(*parentKeyFrame, *this, matchesToKeyFrame);
	if (Ns1<10)
		return false;

	Matcher::PairList f12matchesInliers;
	TTransform motion = Matcher::calculateMovement(*parentKeyFrame, *this, matchesToKeyFrame, f12matchesInliers);
	if (f12matchesInliers.size()<10)
		return false;
	this->setPose(motion);

	matchesToKeyFrame = f12matchesInliers;
	return true;
}


bool
MapBuilder::TmpFrame::track(const kfid &kf)
{
	parentKeyFrame = parent->keyframe(kf);

	int Ns1 = Matcher::matchBruteForce(*parentKeyFrame, *this, matchesToKeyFrame);
	if (Ns1<10)
		return false;

	// Separate old map points (which visible in KF1) from potential new ones (which will be generated by triangulation)
	set<kpid> kp1InMap;
	for (auto &p: parent->allMapPointsAtKeyFrame(kf))
		kp1InMap.insert(p.second);

	for (auto &kpPair: matchesToKeyFrame) {
		if (kp1InMap.find(kpPair.first) != kp1InMap.end()) {
			prevMapPointPairs.push_back(kpPair);
		}
		else {
			candidatesMapPointPairs.push_back(kpPair);
		}
	}

	// XXX: Find solution for insufficient pairs !
	if (prevMapPointPairs.size() < 4) {
		cerr << "Insufficient points!\n";
		return false;
	}

	// Estimate pose for KF2
	Pose PF2;
	Matcher::solvePose(*parentKeyFrame, *this, prevMapPointPairs, PF2);
	this->setPose(PF2);

	return true;
}


bool
MapBuilder::TmpFrame::isOkForKeyFrame() const
{
	const float mapPointThresholdRatio = 0.75;

	if (candidatesMapPointPairs.size() >= matchesToKeyFrame.size()*(1.0-mapPointThresholdRatio))
		return true;

	else return false;
}


KeyFrame::Ptr
MapBuilder::TmpFrame::toKeyFrame() const
{
	auto KF = KeyFrame::fromBaseFrame(*this, parent, 0, this->timestamp);
	KF->setPose(this->pose());
	return KF;
}


MapBuilder::MapBuilder(const CameraPinholeParams &mycam) :
	camera0(mycam)
{
	vMap.reset(new VisionMap);
	vMap->addCameraParameter(camera0);
}


bool
MapBuilder::feed(cv::Mat inputImage, const ptime &timestamp)
{
	currentWorkframe = TmpFrame::create(inputImage, vMap);
	currentWorkframe->timestamp = timestamp;
	frameCounter += 1;

	// No frame yet
	if (lastAnchor==0) {
		auto K1 = KeyFrame::fromBaseFrame(*currentWorkframe, vMap);
		lastAnchor = K1->getId();
		return vMap->addKeyFrame(K1);
	}
	else {

		if (hasInitialized==false) {
			// Try initialization
			if (initialize()==true) {
				hasInitialized = true;
				return true;
			}

			else {
				// skip to next frame, maybe better
				return false;
			}
		}

		// Tracking mode
		else {
			track();
		}
	}

	return true;
}


MapBuilder::~MapBuilder() {
	// TODO Auto-generated destructor stub
}


bool
MapBuilder::initialize()
{
	auto anchorKeyframe = vMap->keyframe(lastAnchor);
	if (currentWorkframe->initializeMatch(anchorKeyframe)==false)
		return false;

	// Create map points
	map<uint, Vector3d> mapPoints;
	float parallax;
	TriangulateCV(*anchorKeyframe, *currentWorkframe, currentWorkframe->matchesToKeyFrame, mapPoints, &parallax);
	if (mapPoints.size() < 10)
		return false;

	auto K2 = KeyFrame::fromBaseFrame(*currentWorkframe, vMap);
	vMap->addKeyFrame(K2);

	// Add points to Map
	for (auto &ptPair: mapPoints) {
		auto inlierKeyPointPair = currentWorkframe->matchesToKeyFrame[ptPair.first];
		auto pt3d = MapPoint::create(ptPair.second);
		vMap->addMapPoint(pt3d);
		vMap->addMapPointVisibility(pt3d->getId(), anchorKeyframe->getId(), inlierKeyPointPair.first);
		vMap->addMapPointVisibility(pt3d->getId(), K2->getId(), inlierKeyPointPair.second);
	}
	cout << "New map initialized: " << mapPoints.size() << " pts\n";
	anchorKeyframe->computeBoW();
	K2->computeBoW();

	vMap->updateCovisibilityGraph(anchorKeyframe->getId());
	lastAnchor = K2->getId();
	return true;
}


bool
MapBuilder::track()
{
	if (currentWorkframe->track(lastAnchor)==false)
		return false;

	/*
	 * It's OK, we only continue when there has been enough 'innovation'
	 */
	if (currentWorkframe->isOkForKeyFrame()==false)
		return true;

	auto Knew = KeyFrame::fromBaseFrame(*currentWorkframe, vMap);
	vMap->addKeyFrame(Knew);

	// Put point appearances
	for (int i=0; i<currentWorkframe->prevMapPointPairs.size(); i++) {
		mpid ptId = vMap->getMapPointByKeypoint(lastAnchor, currentWorkframe->prevMapPointPairs[i].first);
		vMap->addMapPointVisibility(ptId, Knew->getId(), currentWorkframe->prevMapPointPairs[i].second);
		vMap->updateMapPointDescriptor(ptId);
	}

	// Triangulation for new map points
	map<uint, Vector3d> mapPoints;
	float parallax;
	TriangulateCV(*currentWorkframe->parentKeyFrame, *Knew, currentWorkframe->candidatesMapPointPairs, mapPoints, &parallax);
	cout << "Creating map points: " << currentWorkframe->candidatesMapPointPairs.size() << ", got " << mapPoints.size() << endl;
	for (auto &p: mapPoints) {
		auto ptn = MapPoint::create(p.second);
		vMap->addMapPoint(ptn);
		vMap->addMapPointVisibility(ptn->getId(), lastAnchor, currentWorkframe->candidatesMapPointPairs[p.first].first);
		vMap->addMapPointVisibility(ptn->getId(), Knew->getId(), currentWorkframe->candidatesMapPointPairs[p.first].second);
//		vMap->updateMapPointDescriptor(ptn->getId());
	}

	vMap->updateCovisibilityGraph(lastAnchor);

	return true;
}




} /* namespace Vmml */


