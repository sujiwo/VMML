/*
 * ImageDatabaseBuilder.cpp
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#include "Matcher.h"
#include "ImageDatabaseBuilder.h"
#include "utilities.h"
#include "Triangulation.h"


namespace Vmml {

ImageDatabaseBuilder::ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0, const std::string &mapVocabularyPath):
	MapBuilder(camera0, mapVocabularyPath),
	mParams(_p),
	tempLidarCloudmap(new CloudT)
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
{}


/*
 * True when a frame is keyframe, otherwise false
 */
bool
ImageDatabaseBuilder::feed(CloudT::ConstPtr cloudInp, const ptime& cloudTimestamp, cv::Mat img, const ptime& imageTimestamp)
{
	auto frmWork = IdbWorkFrame::create(cloudInp, cloudTimestamp, img, imageTimestamp, vMap->getCameraParameter(0));

	if (anchorFrame==nullptr) {
		lastAnchorLidarPose = Pose::Identity();
		anchorFrame = frmWork;
		anchorFrame->setPose(Pose::Identity());
		addKeyframe(frmWork);

		// Initialize lidar map
		*tempLidarCloudmap += *cloudInp;

		return true;
	}

	TTransform displacement = runNdtMatch(anchorFrame, frmWork);

	bool hasKeyframe=false;
	double linearDispl, angularDispl;
	lastAnchorLidarPose.displacement(frmWork->pose(), linearDispl, angularDispl);
	if (linearDispl>=mParams.min_linear_move or angularDispl>=mParams.min_angular_move) {
		lastAnchorLidarPose = frmWork->pose();
		addKeyframe(frmWork);
		anchorFrame = frmWork;
		hasKeyframe = true;

		// update map
		CloudT transformedScan;
		pcl::transformPointCloud(*frmWork->lidarScan, transformedScan, lastAnchorLidarPose);
		*tempLidarCloudmap += transformedScan;
	}

	return hasKeyframe;
}


void
ImageDatabaseBuilder::addKeyframe(IdbWorkFrame::Ptr kfCandidate)
{
	kfCandidate->setPose(kfCandidate->pose() * lidarToCamera);
	kfCandidate->computeFeatures(vMap->getFeatureDetector());
	auto kfNew = KeyFrame::fromBaseFrame(*kfCandidate, vMap, 0, kfCandidate->imageTimestamp);
	vMap->addKeyFrame(kfNew);
	kfCandidate->keyframeRel = kfNew->getId();

	// Add new map points
	if (kfCandidate!=anchorFrame) {

		Matcher::PairList frameMatchesAtoC;
		int Ns1 = Matcher::matchBruteForce(*anchorFrame, *kfCandidate, frameMatchesAtoC);

		map<uint, Vector3d> mapPoints;
		float parallax;
		TriangulateCV(*anchorFrame, *kfCandidate, frameMatchesAtoC, mapPoints, &parallax);
		for (auto &mpPair: mapPoints) {
			auto nMp = MapPoint::create(mpPair.second);
			vMap->addMapPoint(nMp);
			vMap->addMapPointVisibility(nMp->getId(), anchorFrame->keyframeRel, frameMatchesAtoC[mpPair.first].first);
			vMap->addMapPointVisibility(nMp->getId(), kfCandidate->keyframeRel, frameMatchesAtoC[mpPair.first].second);
		}

		vMap->updateCovisibilityGraph(anchorFrame->keyframeRel);

		// Backtrack (find all map points in anchor frame (and related keyframes) that may be visible
	}

	rigTrack.push_back(PoseStamped(kfCandidate->pose(), kfCandidate->imageTimestamp));
}


/*
 * Returns transformation from previous pose. The frame will be set with current pose
 */
TTransform
ImageDatabaseBuilder::runNdtMatch(IdbWorkFrame::Ptr frame1, IdbWorkFrame::Ptr frame2)
{
	mNdt.setInputTarget(tempLidarCloudmap);

	CloudT::Ptr filteredTargetScan(new CloudT);
	mVoxelGridFilter.setInputCloud(frame2->lidarScan);
	mVoxelGridFilter.filter(*filteredTargetScan);

	mNdt.setInputSource(filteredTargetScan);
	// Guess Pose
	Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
	TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
	Pose guessPose = previousPose * guessDisplacement;

	CloudT::Ptr output_cloud(new CloudT);
	ptime trun1 = getCurrentTime();
	mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
	ptime trun2 = getCurrentTime();
	Pose currentPose = mNdt.getFinalTransformation().cast<double>();

	// Information of lidar matching
	cout << "Converged: " << (mNdt.hasConverged() ? "Y" : "N") << endl;
	cout << "Iteration: " << mNdt.getFinalNumIteration() << endl;
	cout << "Time (s): " << toSeconds(trun2-trun1) << endl;

	lastDisplacement = previousPose.inverse() * currentPose;
	previousPose = currentPose;
	frame2->setPose(currentPose);
	frame2->accumDistance = frame1->accumDistance + lastDisplacement.translation().norm();
	return lastDisplacement;
}

} /* namespace Vmml */
