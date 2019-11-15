/*
 * ImageDatabaseBuilder.cpp
 *
 *  Created on: Nov 15, 2019
 *      Author: sujiwo
 */

#include "ImageDatabaseBuilder.h"


namespace Vmml {

ImageDatabaseBuilder::ImageDatabaseBuilder(Param _p, const CameraPinholeParams &camera0, const std::string &mapVocabularyPath):
	MapBuilder(camera0, mapVocabularyPath),
	mParams(_p)
{}


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
		return;
	}

	TTransform displacement = runMatch(frmWork);

	double linearDispl, angularDispl;
	anchorFrame->pose().displacement(frmWork->pose(), linearDispl, angularDispl);
	if (linearDispl>=mParams.min_linear_move or angularDispl>=mParams.min_angular_move) {
		frmWork->setPose(anchorFrame->pose() * displacement);
		addKeyframe(frmWork);
		anchorFrame = frmWork;
	}
}


void
ImageDatabaseBuilder::addKeyframe(IdbWorkFrame::Ptr keyframe)
{
	keyframe->computeFeatures(vMap->getFeatureDetector());
}


TTransform
ImageDatabaseBuilder::runMatch(IdbWorkFrame::Ptr targetFrame)
{

}

} /* namespace Vmml */
