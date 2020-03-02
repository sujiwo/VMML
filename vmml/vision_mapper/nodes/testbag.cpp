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
#include "vmml/Retinex.h"
#include "Segmentation.h"
#include <opencv2/highgui.hpp>

using namespace std;
using namespace Vmml;

float alpha = 0.3975;


int main(int argc, char *argv[])
{
/*
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag ost(mybag, "/front_rgb/image_raw");

	uint frameNum = stoi(argv[2]);

	auto img0 = ost.at(frameNum),
		imgRaw = ost.at(frameNum, true);
*/
	cv::Mat img0 = cv::imread(argv[1]);

/*
	Mapper::Segmentation
		gSegment("/home/sujiwo/caffe-segnet/segnet_model_driving_webdemo.prototxt",
			"/home/sujiwo/caffe-segnet/segnet_weights_driving_webdemo.caffemodel");

	auto mask = gSegment.buildMask(img0);
*/
	const double retinexSigma[3] = {15.0, 80.0, 250.0};
	Retinex retinex(retinexSigma, 0.01, 0.99);
	auto rtRex = retinex.run(img0);

	cv::imwrite("/tmp/original.png", img0);
	cv::imwrite("/tmp/retinex.png", rtRex);

	return 0;
}
