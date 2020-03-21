/*
 * test_vo.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgproc.hpp>
#include "vmml/VisualOdometry.h"
#include "vmml/utilities.h"
#include "vmml/Matcher.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace Vmml;
using namespace Vmml::Mapper;
using namespace std;


cv::Mat drawOpticalFlow(const BaseFrame::Ptr &anchor, const BaseFrame::Ptr &current, const Matcher::PairList &matches)
{
	if (anchor==nullptr or current==nullptr)
		return cv::Mat();

	cv::Mat myFrame = current->getImage().clone();
	for (auto &pair: matches) {
		auto &kpCurrent = current->keypoint(pair.second);
		auto &kpAnchor = anchor->keypoint(pair.first);
		cv::circle(myFrame, kpCurrent.pt, 2, cv::Scalar(0,255,0));
		cv::circle(myFrame, kpAnchor.pt, 2, cv::Scalar(0,0,255));
		cv::line(myFrame, kpCurrent.pt, kpAnchor.pt, cv::Scalar(255,0,0));
	}

	return myFrame;
}


bool hasBreak = false;
void breakHandler(int sign)
{
	if (sign==SIGINT)
		hasBreak = true;
	cout << "Break is pressed\n";
}


int main(int argc, char *argv[])
{
	float startTimeSeconds=0;
	float maxSecondsFromStart=-1;
	int maxFrameNum=-1;
	float resample=10.0;

	ProgramOptions voProg;
	voProg.addSimpleOptions("start-time", "Mapping will start from x seconds", startTimeSeconds);
	voProg.addSimpleOptions("stop-time", "Maximum seconds from start", maxSecondsFromStart);
	voProg.addSimpleOptions("frames", "Maximum number of frames", maxFrameNum);
	voProg.addSimpleOptions("resample", "Reduce image rate to x Hz", resample);
	voProg.parseCommandLineArgs(argc, argv);

	VisualOdometry::Parameters voPars;
	voPars.camera = voProg.getWorkingCameraParameter();
	auto imagePipe = voProg.getImagePipeline();

	signal(SIGINT, breakHandler);

	VisualOdometry VoRunner(voPars);
	auto imageBag = voProg.getImageBag();
	imageBag->setTimeConstraint(startTimeSeconds, maxSecondsFromStart);

	assert(imagePipe.getOutputSize()==voPars.camera.getImageSize());

	RVizConnector rosConn(argc, argv, "test_vo");

	vector<uint64> targetFrameId;
	imageBag->desample(resample, targetFrameId);
	if (maxFrameNum==-1)
		maxFrameNum = targetFrameId.size();

	for (int n=0; n<maxFrameNum; ++n) {

		auto currentImage = imageBag->at(targetFrameId[n]);
		ptime timestamp = imageBag->timeAt(n).toBoost();

		cv::Mat mask;
		imagePipe.run(currentImage, currentImage, mask);

		VoRunner.process(currentImage, timestamp, mask);

		// Visualization
		auto anchor = VoRunner.getAnchorFrame();
		auto current = VoRunner.getCurrentFrame();
		auto &matches = VoRunner.getLastMatch();

		auto drawFrame = drawOpticalFlow(anchor, current, matches);
		rosConn.publishImage(drawFrame, ros::Time::fromBoost(timestamp));

		if (hasBreak==true)
			break;
	}

	cout << "Done\n";

	const auto voTrack = VoRunner.getTrajectory();
	const auto cloudBuild = VoRunner.getPoints();
	voTrack.dump("/tmp/vo.csv");
	pcl::io::savePCDFileBinary("/tmp/mapVoTest.pcd", *cloudBuild);

	return 0;
}
