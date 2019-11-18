/*
 * ImageDatabaseBuilder.cpp
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#include "ImageDatabaseBuilder.h"
#include "utilities.h"


namespace Vmml {

ImageDatabaseBuilder::ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0, const std::string &mapVocabularyPath):
	MapBuilder(camera0, mapVocabularyPath),
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


ImageDatabaseBuilder::IdbWorkFrame::IdbWorkFrame(CloudT::Ptr &cl, const ptime &lstamp, cv::Mat img, const ptime &istamp, const CameraPinholeParams &cam):
	BaseFrame(img, cam),
	lidarScan(cl),
	lidarTimestamp(lstamp),
	imageTimestamp(istamp)
{}


void
ImageDatabaseBuilder::feed(CloudT::Ptr cloudInp, const ptime& cloudTimestamp, cv::Mat img, const ptime& imageTimestamp)
{
	auto frmWork = IdbWorkFrame::create(cloudInp, cloudTimestamp, img, imageTimestamp, vMap->getCameraParameter(0));

	if (anchorFrame==nullptr) {
		anchorFrame = frmWork;
		anchorFrame->setPose(Pose::Identity());
		addKeyframe(frmWork);
		return;
	}

	TTransform displacement = runMatch(anchorFrame, frmWork);

	double linearDispl, angularDispl;
	anchorFrame->pose().displacement(frmWork->pose(), linearDispl, angularDispl);
	if (linearDispl>=mParams.min_linear_move or angularDispl>=mParams.min_angular_move) {
		frmWork->setPose(anchorFrame->pose() * displacement);
		addKeyframe(frmWork);
		anchorFrame = frmWork;
	}
}


void
ImageDatabaseBuilder::addKeyframe(IdbWorkFrame::Ptr kfCandidate)
{
	kfCandidate->computeFeatures(vMap->getFeatureDetector());
	auto kfNew = KeyFrame::fromBaseFrame(*kfCandidate, vMap, 0, kfCandidate->imageTimestamp);
	vMap->addKeyFrame(kfNew);
}


TTransform
ImageDatabaseBuilder::runMatch(IdbWorkFrame::Ptr frame1, IdbWorkFrame::Ptr frame2)
{
	mNdt.setInputTarget(frame1->lidarScan);

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

	lastDisplacement = previousPose.inverse() * currentPose;
	previousPose = currentPose;
	return lastDisplacement;
}

} /* namespace Vmml */
