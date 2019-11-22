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
using namespace Eigen;


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


KeyFrame::KeyFrame(const std::shared_ptr<VisionMap> _parent) :
	mParent(_parent)
{}


KeyFrame::KeyFrame(cv::Mat img, const std::shared_ptr<VisionMap> _parent, int cameraNo, const ptime &ts, const Pose &p, bool doComputeFeatures) :
	BaseFrame(img, _parent->getCameraParameter(cameraNo), p),
	mParent(_parent),
	cameraId(cameraNo),
	frCreationTime(ts)
{
	id = KeyFrame::nextId;
	if (doComputeFeatures==true) {
		computeFeatures(mParent->getFeatureDetector());
		computeBoW();
	}
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
	VectorXx<float> depths=VectorXx<float>::Zero(allMps.size());
	Matrix4d Cw = externalParamMatrix4();
	Vector3d R2 = Cw.row(2).head(3);
	double zcw = Cw(2,3);

	int i = 0;
	for (auto &ptPair: allMps) {
		auto mapPoint = mParent->mappoint(ptPair.first);
		Vector3d mPos = mapPoint->getPosition();
		double x=mPos.x(), y=mPos.y(), z=mPos.z();
		if (abs(x)<1e-10 or abs(y)<1e-10 or abs(z)<1e-10) {
			int m=1;
		}
		double d = R2.dot(mPos) + zcw;
		depths[i] = d;
		i++;
	}

	return median(depths);
}


} /* namespace Vmml */
