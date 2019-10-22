/*
 * KeyFrame.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include "VisionMap.h"

using namespace std;


namespace Vmml {

/*
 * Ensure that KeyFrame ID is always positive and non-zero
 */
kfid KeyFrame::nextId = 1;


KeyFrame::KeyFrame(cv::Mat img, const std::shared_ptr<VisionMap> _parent, int cameraNo, const Pose &p, bool doComputeFeatures) :
	BaseFrame(img, _parent->getCameraParameter(cameraNo), p),
	mParent(_parent),
	cameraId(cameraNo)
{
	id = KeyFrame::nextId;
	if (doComputeFeatures==true)
		computeFeatures(mParent->getFeatureDetector());
	nextId++;
}


KeyFrame::KeyFrame
(const BaseFrame &bsFrame, const std::shared_ptr<VisionMap> _parent, int cameraNo=0) :
	BaseFrame::BaseFrame(&bsFrame),
	mParent(_parent),
	cameraId(cameraNo)
{
	id = KeyFrame::nextId;
	nextId++;
}


KeyFrame::~KeyFrame() {
	// TODO Auto-generated destructor stub
}


KeyFrame::Ptr
KeyFrame::create(cv::Mat image, const std::shared_ptr<VisionMap>& mParent, int cameraNumber)
{
	Ptr newKf(new KeyFrame(image, mParent, cameraNumber));
	return newKf;
}


KeyFrame::Ptr
KeyFrame::fromBaseFrame(const BaseFrame &frameSource, const std::shared_ptr<VisionMap>& mParent, int cameraNumber)
{
//	auto frm = create(frameSource.getImage(), mParent, cameraNumber);
	Ptr kfrm (new KeyFrame(frameSource.getImage(), mParent, cameraNumber, frameSource.pose(), cameraNumber, false));
	return kfrm;
}

} /* namespace Vmml */
