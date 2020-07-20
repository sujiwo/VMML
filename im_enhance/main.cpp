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
#include "im_enhance.h"


using namespace std;


int main(int argc, char *argv[])
{
/*
	std::vector<int> V = {0, 1, 2, 3, 5};
	auto Vx = matFromIterator<float>(V.begin(), V.end());
	double S = cv::sum(Vx)[0];
	cout << S << endl;
*/
	Eigen::initParallel();

	cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
	cv::Mat res;

	int ch = stoi(argv[2]);
	switch (ch) {
	case 1: res = autoAdjustGammaRGB(image); break;
	case 2: res = multiScaleRetinexCP(image); break;
	case 4: res = exposureFusion(image); break;
	}

//	cv::imwrite("./result.png", res);

	return 0;
}
