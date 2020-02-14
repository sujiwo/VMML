/*
 * testbag.cpp
 *
 *  Created on: Nov 29, 2019
 *      Author: sujiwo
 */

#include <string>
#include <rosbag/bag.h>
#include <pcl/io/pcd_io.h>
#include "vmml/TrajectoryGNSS.h"
#include "vmml/utilities.h"
#include "vmml/LidarScanBag.h"
#include "vmml/ImageBag.h"
#include "vmml/ImagePreprocessor.h"
#include <opencv2/highgui.hpp>

using namespace std;
using namespace Vmml;

float alpha = 0.3975;


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag ost(mybag, "/front_rgb/image_raw");

	uint frameNum = stoi(argv[2]);

	auto img0 = ost.at(frameNum),
		imgRaw = ost.at(frameNum, true);

	const float alpha = 0.3975;
	auto imgIl = ImagePreprocessor::toIlluminatiInvariant(imgRaw, alpha);
	cv::imwrite("/tmp/image-illuminati.png", imgIl);

	auto imgAgc = ImagePreprocessor::autoAdjustGammaRGB(img0);
	cv::imwrite("/tmp/image-agc.png", imgAgc);

	cv::Vec3f
		weights(0.3333, 0.3333, 0.3333),
		sigmas(10, 10, 10);
	auto imgp = ImagePreprocessor::retinaHdr(img0, weights, sigmas, 128, 128, 1.0, 10);
	cv::imwrite("/tmp/image-retinex.png", imgp);

	return 0;
}
