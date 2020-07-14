/*
 * main.cpp
 *
 *  Created on: Jun 25, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/hdf.hpp>
#include <opencv2/core/mat.hpp>
#include "im_enhance.h"


using namespace std;


int main(int argc, char *argv[])
{
/*
	cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
	cv::Mat res;

	int ch = stoi(argv[2]);
	switch (ch) {
	case 1: res = autoAdjustGammaRGB(image); break;
	case 2: res = multiScaleRetinexCP(image); break;
	}

	cv::imwrite("./result.png", res);
*/
	Eigen::Matrix<int,2,4> zzz;
	zzz <<
		1, 2, 3, 4,
		-4, -3, -2, -1;
//	cout << zzz << endl;

	Eigen::Vector2i diagsl;
	diagsl << 0, 2;
	auto sps = spdiags(zzz, diagsl, 4, 4);
	cout << sps << endl;

	return 0;
}
