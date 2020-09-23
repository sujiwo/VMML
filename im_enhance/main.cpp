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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/hdf.hpp>
#include <boost/filesystem.hpp>
#include "mpi.h"
#include "im_enhance.h"
#include "npy.hpp"
#include "src/matutils.h"
#include "src/timer.h"


using namespace std;
namespace fs=boost::filesystem;



int main(int argc, char *argv[])
{
	// This line may not be required outside MUMPS
//	MPI_Init(&argc, &argv);

	fs::path inputImage(argv[1]);

	cv::Mat image = cv::imread(inputImage.string(), cv::IMREAD_COLOR);
	cv::Mat res;

	const float alpha = 0.3975;

	auto t1 = ice::getCurrentTime();
	int ch = stoi(argv[2]);
	switch (ch) {
	case 1: res = ice::autoAdjustGammaRGB(image); break;
	case 2: res = ice::multiScaleRetinexCP(image); break;
	case 3: res = ice::toIlluminatiInvariant(image, alpha); break;
	case 4: res = ice::exposureFusion(image); break;
	case 5: res = ice::dynamicHistogramEqualization(image); break;
	}
	auto t2 = ice::getCurrentTime();
	cout << "Preprocessing time: " << ice::to_seconds(t2-t1) << endl;

	fs::path outputImage(inputImage.parent_path() / (inputImage.stem().string()+'-'+to_string(ch)+inputImage.extension().string()));
	cv::imwrite(outputImage.string(), res);

	return 0;
}
