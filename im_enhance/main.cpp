/*
 * main.cpp
 *
 *  Created on: Jun 25, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/hdf.hpp>
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
	cv::Mat A(3, 3, CV_8UC1);
	cv::randu(A, cv::Scalar::all(0), cv::Scalar::all(255));
	cout << A << endl;

	auto As = flatten(A, 1);
	cout << As << endl;

	return 0;
}
