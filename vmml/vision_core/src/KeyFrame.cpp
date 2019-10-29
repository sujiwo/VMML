/*
 * KeyFrame.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#include "utilities.h"
#include "KeyFrame.h"
#include "VisionMap.h"

using namespace std;


namespace Vmml {

/*
 * Ensure that KeyFrame ID is always positive and non-zero
 */
kfid KeyFrame::nextId = 1;


std::vector<cv::Mat>
KeyFrame::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}


KeyFrame::KeyFrame(cv::Mat img, const std::shared_ptr<VisionMap> _parent, int cameraNo, const ptime &ts, const Pose &p, bool doComputeFeatures) :
	BaseFrame(img, _parent->getCameraParameter(cameraNo), p),
	mParent(_parent),
	cameraId(cameraNo),
	frCreationTime(ts)
{
	id = KeyFrame::nextId;
	if (doComputeFeatures==true)
		computeFeatures(mParent->getFeatureDetector());
	nextId++;
}


KeyFrame::KeyFrame
(const BaseFrame &bsFrame, const std::shared_ptr<VisionMap> _parent, int cameraNo, const ptime &ts) :
	BaseFrame::BaseFrame(bsFrame),
	mParent(_parent),
	cameraId(cameraNo),
	frCreationTime(ts)
{
	id = KeyFrame::nextId;
	nextId++;
}


KeyFrame::~KeyFrame() {
	// TODO Auto-generated destructor stub
}


KeyFrame::Ptr
KeyFrame::create(cv::Mat image, const std::shared_ptr<VisionMap>& mParent, int cameraNumber, const ptime &ts)
{
	Ptr newKf(new KeyFrame(image, mParent, cameraNumber, ts));
	newKf->frCreationTime = ts;
	return newKf;
}


KeyFrame::Ptr
KeyFrame::fromBaseFrame(const BaseFrame &frameSource, const std::shared_ptr<VisionMap>& mParent, int cameraNumber, const ptime &ts)
{
	Ptr kfrm (new KeyFrame(frameSource, mParent, cameraNumber));
	kfrm->frCreationTime = ts;
	return kfrm;
}


void
KeyFrame::computeBoW()
{
	if (mParent->BoWList[this->id].empty() or mParent->FeatVecList[this->id].empty()) {
		vector<cv::Mat> kfDescs = toDescriptorVector(allDescriptors());

		// Build BoW descriptor of this keyframe
		mParent->BoWList[this->id] = DBoW2::BowVector();
		mParent->FeatVecList[this->id] = DBoW2::FeatureVector();
		mParent->myVoc.transform(kfDescs, mParent->BoWList[this->id], mParent->FeatVecList[this->id], 4);

		// Build Inverse Index
		for (auto &bowvec: mParent->BoWList[this->id]) {
			const DBoW2::WordId wrd = bowvec.first;
			mParent->invertedKeywordDb[wrd].insert(this->id);
		}
	}
}


double
KeyFrame::computeSceneMedianDepth() const
{
	auto allMps = mParent->allMapPointsAtKeyFrame(id);
	VectorXx<float> depths(allMps.size());
	Vector3d R2 = externalParamMatrix4().row(2).head(3);
	double zcw = externalParamMatrix4()(2,3);

	int i = 0;
	for (auto &ptPair: allMps) {
		auto mapPoint = mParent->mappoint(ptPair.first);
		auto d = R2.dot(mapPoint->getPosition()) + zcw;
		depths[i] = d;
	}

	return median(depths);
}


} /* namespace Vmml */
