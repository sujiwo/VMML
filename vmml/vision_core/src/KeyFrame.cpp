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


KeyFrame::KeyFrame(cv::Mat img, const std::shared_ptr<VisionMap> _parent, int cameraNo, const Pose &p) :
	BaseFrame(img, _parent->getCameraParameter(cameraNo), p),
	mParent(_parent),
	cameraId(cameraNo)
{
	id = KeyFrame::nextId;
	computeFeatures(mParent->getFeatureDetector());
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
	return create(frameSource.getImage(), mParent, cameraNumber);
}

} /* namespace Vmml */
