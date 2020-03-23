/*
 * viso_run.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: sujiwo
 */

#include <vector>
#include <iostream>
#include <cstdio>
#include "viso_mono.h"
#include "vmml/Pose.h"
#include "vmml/Trajectory.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;


int main(int argc, char *argv[])
{
	float startTimeSeconds=0;
	float maxSecondsFromStart=-1;
	float resample=10.0;

	Vmml::Mapper::ProgramOptions vsoProg;
	vsoProg.addSimpleOptions("start-time", "Mapping will start from x seconds", startTimeSeconds);
	vsoProg.addSimpleOptions("stop-time", "Maximum seconds from start", maxSecondsFromStart);
	vsoProg.addSimpleOptions("resample", "Reduce image rate to x Hz", resample);
	vsoProg.parseCommandLineArgs(argc, argv);

	auto imageBag = vsoProg.getImageBag();
	auto cameraPars = vsoProg.getWorkingCameraParameter();
	auto &imagePipe = vsoProg.getImagePipeline();

	imageBag->setTimeConstraint(startTimeSeconds, maxSecondsFromStart);
	RandomAccessBag::DesampledMessageList targetFrameId;
	imageBag->desample(resample, targetFrameId);

	// VISO setup
	VisualOdometryMono::parameters visoMonoPars;
	visoMonoPars.calib.f = (cameraPars.fx+cameraPars.fy)/2;
	visoMonoPars.calib.cu = cameraPars.cx;
	visoMonoPars.calib.cv = cameraPars.cy;

	VisualOdometryMono visoRun(visoMonoPars);
	Vmml::Trajectory voTrack;
	bool firstRun = true;

	Vmml::Mapper::RVizConnector rosConn(argc, argv, "test_vo");
	rosConn.setImageTopicName("viso");

	for (auto i: targetFrameId) {

		cv::Mat image = imageBag->at(i);
		auto timestamp = imageBag->timeAt(i);
		imagePipe.run(image, image);
		int32_t dims[] = {image.cols, image.rows, image.step};

		if (firstRun) {
			visoRun.process(image.data, dims);
			voTrack.push_back(Vmml::PoseStamped::Identity(timestamp.toBoost()));
			firstRun = false;
		}

		else {
			bool success = visoRun.process(image.data, dims);

			if (success) {
				auto camera_motion = visoRun.getMotion();
				// WHY Inverse ?
				camera_motion.inv();
				printf("%d: Found %d matches with %d inliers\n",
					visoRun.getNumberOfMatches(),
					visoRun.getNumberOfInliers());

				Eigen::Matrix3d rot_mat;
				rot_mat <<
					camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
					camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
					camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2];
				Eigen::Vector3d t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
				auto cTrans = Vmml::TTransform::from_R_t(t, rot_mat);
				auto cpose = voTrack.back() * cTrans;
				voTrack.push_back(Vmml::PoseStamped(cpose, timestamp.toBoost()));
			}

			else {
				cerr << "Odometry failed; assuming motionless\n";
				Vmml::PoseStamped px = voTrack.back();
				px.timestamp = timestamp.toBoost();
				voTrack.push_back(px);
			}
		}

		rosConn.publishImage(image, timestamp);
		cout << i << '/' << targetFrameId.size() << endl;
	}

	voTrack.dump("/tmp/viso.csv");
	return 0;
}
