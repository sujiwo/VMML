/*
 * image.cpp
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#include "VisionMap.h"
#include "ImageBag.h"
#include "BaseFrame.h"
#include "Matcher.h"


using namespace std;
using namespace Vmml;

CameraPinholeParams camera0 (
	1150.96938467,
	1150.96938467,
	988.511326762,
	692.803953253,
	1920, 1440);
float enlarge = 0.333333333333;

VisionMap testMap;

int main(int argc, char *argv[])
{
	camera0 = camera0 * enlarge;

	rosbag::Bag inputBag("/Data/MapServer/Logs/campus_loop/campus_loop.bag");
	ImageBag images(inputBag, "/camera1/image_raw", enlarge);

	auto frame1 = BaseFrame::create(images.at(180), Pose::Identity(), camera0);
	auto frame2 = BaseFrame::create(images.at(181), Pose::Identity(), camera0);

	Matcher::PairList matches12;
	frame1->computeFeatures(testMap.getFeatureDetector());
	frame2->computeFeatures(testMap.getFeatureDetector());
	int n = Matcher::matchForInitialization(*frame1, *frame2, matches12);

	return 0;
}


