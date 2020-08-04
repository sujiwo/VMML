/*
 * main.cpp
 *
 *  Created on: Jun 25, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/hdf.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/ocl.hpp>
#include <boost/filesystem.hpp>
#include "im_enhance.h"
#include "npy.hpp"


using namespace std;
namespace fs=boost::filesystem;


int main(int argc, char *argv[])
{
	Eigen::initParallel();

	fs::path inputImage(argv[1]);

	cv::Mat image = cv::imread(inputImage.string(), cv::IMREAD_COLOR);
	cv::Mat res;

	const float alpha = 0.3975;

	int ch = stoi(argv[2]);
	switch (ch) {
	case 1: res = autoAdjustGammaRGB(image); break;
	case 2: res = multiScaleRetinexCP(image); break;
	case 3: res = toIlluminatiInvariant(image, alpha); break;
	case 4: res = exposureFusion(image); break;
	}

	fs::path outputImage(inputImage.parent_path() / (inputImage.stem().string()+'-'+to_string(ch)+inputImage.extension().string()));
	cv::imwrite(outputImage.string(), res);

	return 0;
}
