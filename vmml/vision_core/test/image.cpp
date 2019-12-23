/*
 * image.cpp
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include "vmml/VisionMap.h"
#include "vmml/ImageBag.h"
#include "vmml/LidarScanBag.h"
#include "vmml/BaseFrame.h"
#include "vmml/Matcher.h"


using namespace std;
using namespace Vmml;

CameraPinholeParams camera0 (
	1150.96938467,
	1150.96938467,
	988.511326762,
	692.803953253,
	1920, 1440);
float enlarge = 0.666666666667;

// XXX: Supply your own values!
const TTransform tLidarToCamera = TTransform::from_XYZ_RPY(
	Eigen::Vector3d(0.9, 0.3, -0.6),
	-1.520777, -0.015, -1.5488);

VisionMap testMap;

int main(int argc, char *argv[])
{
	/*
	 * please run `setup install/setup.bash' prior to run this program
	 */
	auto maskPath = getMyPath() / "samples/meidai_mask.png";
	camera0.mask = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
	camera0 = camera0 * enlarge;

	rosbag::Bag inputBag(argv[1]);
	ImageBag images(inputBag, "/camera1/image_raw", enlarge);
	images.setGammaMeteringMask();
	LidarScanBag pcdScans(inputBag, "/velodyne_packets");

	int i1 = stoi(argv[2]);
	auto t1 = images.timeAt(i1);
	auto l1 = pcdScans.getPositionAtTime(t1);
	auto imageFrame1 = BaseFrame::create(images.getGrayscale(i1), camera0);
	imageFrame1->computeFeatures(testMap.getFeatureDetector());
	auto lidarScan1 = pcdScans.getUnfiltered(l1);

/*
	map<uint32_t, uint32_t> imageToLidar;
	imageFrame1->associateToLidarScans(*lidarScan1, tLidarToCamera, imageToLidar);
*/
	auto projRes = imageFrame1->projectLidarScan(*lidarScan1, tLidarToCamera);
	cv::Mat projImg;
	cv::cvtColor(imageFrame1->getImage(), projImg, CV_GRAY2BGR);
	for (auto &p: projRes) {
		cv::circle(projImg, cv::Point2f(p.x, p.y), 1.5, cv::Scalar(255,0,0));
	}

	cv::imwrite(string(argv[2])+".png", projImg);

	return 0;
}


