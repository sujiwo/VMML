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
#include <boost/filesystem.hpp>
#include <boost/math/tools/minima.hpp>
#include "im_enhance.h"
#include "npy.hpp"


using namespace std;
namespace fs=boost::filesystem;


int main(int argc, char *argv[])
{
	fs::path inputImage(argv[1]);

	cv::Mat image = cv::imread(inputImage.string(), cv::IMREAD_COLOR);
	cv::Mat res;

	const float alpha = 0.3975;

	int ch = stoi(argv[2]);
	switch (ch) {
	case 1: res = ice::autoAdjustGammaRGB(image); break;
	case 2: res = ice::multiScaleRetinexCP(image); break;
	case 3: res = ice::toIlluminatiInvariant(image, alpha); break;
	case 4: res = ice::exposureFusion(image); break;
	}

	fs::path outputImage(inputImage.parent_path() / (inputImage.stem().string()+'-'+to_string(ch)+inputImage.extension().string()));
	cv::imwrite(outputImage.string(), res);
/*
	const float
		a_ = -0.3293,
		b_ = 1.1258;

	ice::Matf Yx = npy::loadMat("/tmp/Y.npy");

	// define functions
	auto funEntropy = [&](float k)->float {
		return -ice::entropy(ice::applyK(Yx, k, a_, b_));
	};
	auto fmin = boost::math::tools::brent_find_minima(funEntropy, 1.0, 7.0, numeric_limits<float>::digits10);
*/

	return 0;
}
