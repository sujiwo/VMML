/*
 * preprocess_image_bag.cpp
 *
 *  Created on: Sep 8, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <opencv2/imgproc.hpp>
#include "ProgramOptions.h"
#include "RandomAccessBag.h"
#include "im_enhance.h"


using namespace std;
namespace fs=boost::filesystem;


int main(int argc, char *argv[])
{
	int ch;
	fs::path outputPath, inputPath;

	Vmml::Mapper::ProgramOptions po;
	po.addSimpleOptions("method", "Image preprocessing method", &ch, true);
	po.addSimpleOptions("output", "Output bag file name", &outputPath);
	po.parseCommandLineArgs(argc, argv);

	return 0;
}
