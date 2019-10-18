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


MapBuilder::MapBuilder(const CameraPinholeParams &mycam) :
	camera0(mycam)
{
	vMap.reset(new VisionMap);
	vMap->addCameraParameter(camera0);
}


bool
MapBuilder::feed(cv::Mat inputImage)
{
	auto currentFrame = BaseFrame::create(inputImage, camera0);

	if (lastAnchor==0) {
		auto K1 = KeyFrame::fromBaseFrame(*currentFrame, vMap);
		lastAnchor = K1->getId();
		return vMap->addKeyFrame(K1);
	}
	else {
		initialize(currentFrame);
	}

	return true;
}


MapBuilder::~MapBuilder() {
	// TODO Auto-generated destructor stub
}


bool
MapBuilder::initialize(BaseFrame::Ptr &f2)
{
	auto K1 = vMap->keyframe(lastAnchor);
	Matcher::PairList f12matches, f12matchesInliers;

	int Ns1 = Matcher::matchBruteForce(*K1, *f2, f12matches);
	if (Ns1<10)
		return false;
	TTransform motion = Matcher::calculateMovement(*K1, *f2, f12matches, f12matchesInliers);
	if (f12matchesInliers.size()<10)
		return false;
	f2->setPose(motion);

	map<uint, Vector3d> mapPoints;
	TriangulateCV(*K1, *f2, f12matchesInliers, mapPoints);



	return true;
}


} /* namespace Vmml */


