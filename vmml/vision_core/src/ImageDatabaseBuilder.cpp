/*
 * ImageDatabaseBuilder.cpp
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#include "vmml/Matcher.h"
#include "vmml/ImageDatabaseBuilder.h"
#include "vmml/utilities.h"
#include "vmml/Triangulation.h"


namespace Vmml {

ImageDatabaseBuilder::ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0):
	SimpleMapBuilder(camera0),
	mParams(_p)
{
	// Parameter set
	mNdt.setResolution(mParams.ndt_res);
	mNdt.setStepSize(mParams.step_size);
	mNdt.setTransformationEpsilon(mParams.trans_eps);
	mNdt.setMaximumIterations(mParams.max_iter);

	mVoxelGridFilter.setLeafSize(mParams.voxel_leaf_size, mParams.voxel_leaf_size, mParams.voxel_leaf_size);
}


ImageDatabaseBuilder::~ImageDatabaseBuilder()
{
	// TODO Auto-generated destructor stub
}


ImageDatabaseBuilder::IdbWorkFrame::IdbWorkFrame(CloudT::ConstPtr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam):
	BaseFrame(img, cam),
	lidarScan(cl),
	lidarTimestamp(lstamp),
	imageTimestamp(istamp)
{
	setPose(Pose::Identity());
}


/*
 * True when a frame is keyframe, otherwise false
 */
bool
ImageDatabaseBuilder::feed(CloudT::ConstPtr cloudInp, const ptime& cloudTimestamp, cv::Mat img, const ptime& imageTimestamp)
{
	auto frmWork = IdbWorkFrame::create(cloudInp, cloudTimestamp, img, imageTimestamp, vMap->getCameraParameter(0));

	if (anchorFrame==nullptr) {
		frmWork->isKeyFrame = true;
		anchorFrame = frmWork;
		lastFrame = frmWork;
		addKeyframe(frmWork);
		rigTrack.push_back(PoseStamped(Pose::Identity(), cloudTimestamp));
		return true;
	}

	TTransform displacement = runNdtMatch(lastFrame, frmWork);

	bool hasKeyframe=false;
	double linearDispl, angularDispl;
	anchorFrame->pose().displacement(frmWork->pose(), linearDispl, angularDispl);
	if (linearDispl>=mParams.min_linear_move or angularDispl>=mParams.min_angular_move) {
		frmWork->isKeyFrame = true;
		addKeyframe(frmWork);
		anchorFrame = frmWork;
		hasKeyframe = true;
	}

	else {
		frmWork->keyframeRel = anchorFrame->keyframeRel;
	}

	lastFrame = frmWork;

	rigTrack.push_back(PoseStamped(frmWork->pose(), cloudTimestamp));
	return hasKeyframe;
}


void
ImageDatabaseBuilder::addKeyframe(IdbWorkFrame::Ptr kfCandidate)
{
	// kfCandidate has lidar pose. transform to camera pose for keyframe
	Pose ldPos = kfCandidate->pose();
	kfCandidate->computeFeatures(vMap->getFeatureDetector());
	auto kfNew = KeyFrame::fromBaseFrame(*kfCandidate, vMap, 0, kfCandidate->imageTimestamp);
	kfNew->setPose(ldPos * lidarToCamera);
	vMap->addKeyFrame(kfNew);
	kfCandidate->keyframeRel = kfNew->getId();

	// Add new map points
	if (kfCandidate!=anchorFrame) {

		auto anchorKey = vMap->keyframe(anchorFrame->keyframeRel);
		Matcher::PairList frameMatchesAtoC;
		int Ns1 = Matcher::matchBruteForce(*anchorKey, *kfNew, frameMatchesAtoC);

		map<uint, Vector3d> mapPoints;
		float parallax;
		TriangulateCV(*anchorKey, *kfNew, frameMatchesAtoC, mapPoints, &parallax);
		for (auto &mpPair: mapPoints) {
			auto nMp = MapPoint::create(mpPair.second);
			vMap->addMapPoint(nMp);
			vMap->addMapPointVisibility(nMp->getId(), anchorFrame->keyframeRel, frameMatchesAtoC[mpPair.first].first);
			vMap->addMapPointVisibility(nMp->getId(), kfCandidate->keyframeRel, frameMatchesAtoC[mpPair.first].second);
		}

		vMap->updateCovisibilityGraph(anchorFrame->keyframeRel);

		// Backtrack (find all map points in anchor frame (and related keyframes) that may be visible
		// Build connections to previous keyframes
		vector<kfid> kfInsToAnchor = vMap->getKeyFramesComeInto(anchorFrame->keyframeRel);
		for (auto kf1: kfInsToAnchor) {
			trackMapPoints(kf1, kfCandidate->keyframeRel);
		}

		kfCandidate->featureMatchesFromLastAnchor = frameMatchesAtoC;
	}
}


/*
 * Returns transformation from previous pose. The frame will be set with current lidar pose
 */
TTransform
ImageDatabaseBuilder::runNdtMatch(IdbWorkFrame::Ptr frame1, IdbWorkFrame::Ptr frame2)
{
	mNdt.setInputTarget(frame1->lidarScan);

	CloudT::Ptr filteredTargetScan(new CloudT);
	mVoxelGridFilter.setInputCloud(frame2->lidarScan);
	mVoxelGridFilter.filter(*filteredTargetScan);
	mNdt.setInputSource(filteredTargetScan);

	CloudT output_cloud;
	ptime trun1 = getCurrentTime();
	mNdt.align(output_cloud);
	ptime trun2 = getCurrentTime();
	TTransform movementByLidar = mNdt.getFinalTransformation().cast<double>();

	frame2->setPose(frame1->pose() * movementByLidar);
	frame2->accumDistance = frame1->accumDistance + movementByLidar.translation().norm();
	return movementByLidar;
}

} /* namespace Vmml */
