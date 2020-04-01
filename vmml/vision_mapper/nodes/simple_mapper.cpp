/*
 * simple_bag_mapper.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <vmml/SimpleMapBuilder.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "RVizConnector.h"
#include "ImagePipeline.h"
#include "ProgramOptions.h"


using namespace std;
using namespace std::placeholders;
using namespace Vmml;
using namespace Vmml::Mapper;


void publish(RVizConnector &ros, SimpleMapBuilder::TmpFrame::Ptr &currentFrame)
{
	auto currentPose = currentFrame->pose();
	auto rstamp = ros::Time::fromBoost(currentFrame->timestamp);
	ros.publishImage(currentFrame->visualize(), rstamp);
}


int main(int argc, char *argv[])
{
	float startTimeSeconds=0;
	float maxSecondsFromStart=-1;
	float resample=10.0;

	RVizConnector rosConn(argc, argv, "test_vo");

	ProgramOptions mapperProg;
	mapperProg.addSimpleOptions("start-time", "Mapping will start from x seconds", &startTimeSeconds);
	mapperProg.addSimpleOptions("stop-time", "Maximum seconds from start", &maxSecondsFromStart);
	mapperProg.addSimpleOptions("resample", "Reduce image rate to x Hz", &resample);
	mapperProg.parseCommandLineArgs(argc, argv);

	auto imageBag = mapperProg.getImageBag();
	imageBag->setTimeConstraint(startTimeSeconds, maxSecondsFromStart);

	RandomAccessBag::DesampledMessageList targetFrameId;
	imageBag->desample(resample, targetFrameId);

	SimpleMapBuilder::Parameters mapBuilderParams;
	mapBuilderParams.camera = mapperProg.getWorkingCameraParameter();
	SimpleMapBuilder mapBuild(mapBuilderParams);

	mapBuilderParams.camera = mapperProg.getWorkingCameraParameter();
	auto imagePipe = mapperProg.getImagePipeline();

	for (int n=0; n<targetFrameId.size(); ++n) {

		auto currentImage = imageBag->at(targetFrameId[n]);
		ptime timestamp = imageBag->timeAt(n).toBoost();

		cv::Mat mask;
		imagePipe.run(currentImage, currentImage, mask);

		mapBuild.process(currentImage, timestamp, mask);

		auto curFrame = mapBuild.getCurrentFrame();
		publish(rosConn, curFrame);
	}

	mapBuild.end();
	auto trajectory=mapBuild.getTrajectory();
	trajectory.dump("/tmp/simple-vslam.csv");

	return 0;
}
