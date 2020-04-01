/*
 * vosl_demo.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: sujiwo
 */


#include <string>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/publish/frame_publisher.h"
#include "openvslam/publish/map_publisher.h"
#include "openvslam/data/landmark.h"

#include "vmml/Pose.h"
#include "vmml/Trajectory.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;


YAML::Node createDummyConfig(const Vmml::Mapper::ProgramOptions &po, float resample=-1)
{
	YAML::Node vsConf;

	auto cameraConf = po.getWorkingCameraParameter();
	vsConf["Camera.name"] = "Camera0";
	vsConf["Camera.setup"] = "monocular";
	vsConf["Camera.model"] = "perspective";
	vsConf["Camera.fx"] = cameraConf.fx;
	vsConf["Camera.fy"] = cameraConf.fy;
	vsConf["Camera.cx"] = cameraConf.cx;
	vsConf["Camera.cy"] = cameraConf.cy;
	vsConf["Camera.k1"] = cameraConf.distortionCoeffs[0];
	vsConf["Camera.k2"] = cameraConf.distortionCoeffs[1];
	vsConf["Camera.p1"] = cameraConf.distortionCoeffs[2];
	vsConf["Camera.p2"] = cameraConf.distortionCoeffs[3];
	vsConf["Camera.k3"] = cameraConf.distortionCoeffs[4];
	vsConf["Camera.fps"] = resample;
	vsConf["Camera.cols"] = cameraConf.width;
	vsConf["Camera.rows"] = cameraConf.height;
	vsConf["Feature.max_num_keypoints"] = 6000;
	vsConf["Feature.scale_factor"] = 1.2;
	vsConf["Feature.num_levels"] = 8;
	vsConf["Feature.ini_fast_threshold"] = 20;
	vsConf["Feature.min_fast_threshold"] = 7;
	vsConf["Camera.color_order"] = "BGR";

	return vsConf;
}


class PrimitiveViewer
{
public:
PrimitiveViewer(Vmml::Mapper::ProgramOptions &prog, openvslam::system &slam_):
	slam(slam_),
	mapPub(slam_.get_map_publisher()),
	framePub(slam_.get_frame_publisher()),
	rosConn(prog.getArgc(), prog.getArgv(), "vosl_demo")
{
	rosConn.setImageTopicName("openvslam");
}

void run()
{

}

void publishMap(const ros::Time &timestamp)
{
	std::vector<openvslam::data::keyframe*> keyfrms;
	mapPub->get_keyframes(keyfrms);

	vector<Pose> track;
	for (auto kf: keyfrms) {
		Pose pkf = kf->get_cam_pose_inv();
		track.push_back(pkf);
	}

	std::vector<openvslam::data::landmark*> landmarks;
	std::set<openvslam::data::landmark*> local_landmarks;
	mapPub->get_landmarks(landmarks, local_landmarks);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mapToCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto &lm: landmarks) {
		auto pt = lm->get_pos_in_world();
		pcl::PointXYZ pt3d;
		pt3d.x = pt.x();
		pt3d.y = pt.y();
		pt3d.z = pt.z();
		mapToCloud->push_back(pt3d);
	}

	rosConn.publishTrajectory(track, timestamp);
	rosConn.publishPointCloud(mapToCloud, timestamp);
}

void publishFrame(const ros::Time &timestamp)
{
	cv::Mat frame = framePub->draw_frame();
	Eigen::Matrix4d posee = mapPub->get_current_cam_pose().inverse();
	Pose pose = posee;
	rosConn.publishImage(frame, timestamp);
}

private:
	openvslam::system &slam;
	const shared_ptr<openvslam::publish::frame_publisher> framePub;
	const shared_ptr<openvslam::publish::map_publisher> mapPub;
	Vmml::Mapper::RVizConnector rosConn;
};


/////////////////

int main(int argc, char *argv[])
{
	Vmml::Mapper::ProgramOptions vsoProg;
	vsoProg.addSimpleOptions("vocabulary", "Path to vocabulary", true);
	vsoProg.addSimpleOptions<float>("start-time", "Mapping will start from x seconds");
	vsoProg.addSimpleOptions<float>("stop-time", "Maximum seconds from start");
	vsoProg.addSimpleOptions<float>("resample", "Reduce image rate to x Hz");
	vsoProg.parseCommandLineArgs(argc, argv);

	auto imageBag = vsoProg.getImageBag();
	auto cameraPars = vsoProg.getWorkingCameraParameter();
	auto &imagePipe = vsoProg.getImagePipeline();

	auto vocPath = vsoProg.get<string>("vocabulary", "");
	auto startTime = vsoProg.get<float>("start-time", 0);
	auto stopTime = vsoProg.get<float>("stop-time", -1);
	auto resample = vsoProg.get<float>("resample", 0);

	if (resample==0)
		resample = imageBag->hz();
	imageBag->setTimeConstraint(startTime, stopTime);
	RandomAccessBag::DesampledMessageList targetFrameId;
	imageBag->desample(resample, targetFrameId);

	auto dummyYaml = createDummyConfig(vsoProg, resample);
	auto slamConfig = make_shared<openvslam::config>(dummyYaml);

	// Slam dunk
	openvslam::system SlamDunk (slamConfig, vocPath);
	SlamDunk.startup();
	PrimitiveViewer rosConn(vsoProg, SlamDunk);

	for (auto &frameId: targetFrameId) {

		auto image = imageBag->at(frameId);
		auto timestamp = imageBag->timeAt(frameId);
		cv::Mat mask;
		imagePipe.run(image, image, mask);

		SlamDunk.feed_monocular_frame(image, timestamp.toSec(), mask);

		auto framePub = SlamDunk.get_frame_publisher();
		auto mapPub = SlamDunk.get_map_publisher();

		// XXX: temporary
		cv::Mat frame = framePub->draw_frame();
		rosConn.publishFrame(timestamp);
		rosConn.publishMap(timestamp);

		if (SlamDunk.terminate_is_requested())
			break;
	}

	SlamDunk.shutdown();
	return 0;
}
