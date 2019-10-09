/*
 * KeyFrame.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#include <KeyFrame.h>

namespace Vmml {

/*
 * Ensure that KeyFrame ID is always positive and non-zero
 */
kfid KeyFrame::nextId = 1;


KeyFrame::KeyFrame() {
	// TODO Auto-generated constructor stub

}


KeyFrame::~KeyFrame() {
	// TODO Auto-generated destructor stub
}


KeyFrame::Ptr
KeyFrame::create(cv::Mat image, const std::shared_ptr<VisionMap>& mParent, int cameraNumber)
{
	Ptr newKf(new KeyFrame);
	newKf->mParent = mParent;
	newKf->image = image;
	newKf->cameraParam = mParent->getCameraParameter(cameraNumber);
	newKf->computeFeatures(mParent->getFeatureDetector());
	return newKf;
}


static Ptr
KeyFrame::fromBaseFrame(const BaseFrame &frameSource, const std::shared_ptr<VisionMap>& mParent, int cameraNumber)
{

}

} /* namespace Vmml */
