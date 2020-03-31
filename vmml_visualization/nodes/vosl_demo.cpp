/*
 * vosl_demo.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: sujiwo
 */


#include <string>
#include <vector>
#include <iostream>
#include "openvslam/system.h"
#include "vmml/Pose.h"
#include "vmml/Trajectory.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;


int main(int argc, char *argv[])
{
	Vmml::Mapper::ProgramOptions vsoProg;
	vsoProg.addSimpleOptions("vocabulary", "Path to vocabulary");
	vsoProg.addSimpleOptions("start-time", "Mapping will start from x seconds");
	vsoProg.addSimpleOptions("stop-time", "Maximum seconds from start");
	vsoProg.addSimpleOptions("resample", "Reduce image rate to x Hz");
	vsoProg.parseCommandLineArgs(argc, argv);

	auto imageBag = vsoProg.getImageBag();
	auto cameraPars = vsoProg.getWorkingCameraParameter();
	auto &imagePipe = vsoProg.getImagePipeline();

	auto vocPath = vsoProg.get<string>("vocabulary");
	auto startTime = vsoProg.get<float>("start-time");
	auto stopTime = vsoProg.get<float>("stop-time");
	auto resample = vsoProg.get<float>("resample");

	return 0;
}
