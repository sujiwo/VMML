/*
 * vosl_demo.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: sujiwo
 */

#include <unistd.h>
#include <signal.h>
#include <string>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/publish/frame_publisher.h"
#include "openvslam/publish/map_publisher.h"
#include "openvslam/data/landmark.h"

#include "vmml/Pose.h"
#include "vmml/Trajectory.h"
#include "vmml/TrajectoryGNSS.h"
#include "ProgramOptions.h"
#include "ROSConnector.h"


using namespace std;
using namespace Vmml;


const TTransform rot180(0, 0, 0, -M_PI_2, 0, 0);
const TTransform rot180x(0, 0, 0, 1.571, 0, 0);
bool breakImageStream = false;

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
	vsConf["Feature.max_num_keypoints"] = po.getMaxOrbKeypoints();
	vsConf["Feature.scale_factor"] = 1.2;
	vsConf["Feature.num_levels"] = 8;
	vsConf["Feature.ini_fast_threshold"] = 20;
	vsConf["Feature.min_fast_threshold"] = 7;
	vsConf["Camera.color_order"] = "BGR";

	return vsConf;
}


void handleInterruptSignal(int signal)
{
	breakImageStream = true;
}


class PrimitiveViewer
{
using PubId=Mapper::ROSConnector::PublisherId;

public:
PrimitiveViewer(Vmml::Mapper::ProgramOptions &prog, openvslam::system &slam_):
	slam(slam_),
	mapPub(slam_.get_map_publisher()),
	framePub(slam_.get_frame_publisher()),
	rosConn(prog.getArgc(), prog.getArgv(), "vosl_demo", ros::InitOption::NoSigintHandler)
{
	cout << "Start\n";
	camera = prog.getWorkingCameraParameter();
	useRealtime = prog.get<bool>("ros-time", false);
	pubOVFrame = rosConn.createImagePublisher("frame_render", prog.getWorkingCameraParameter(), "camera");
	pubFramePose = rosConn.createPosePublisher("world", "camera");
	cout << "XYZ 1\n";
	pubCamTrack = rosConn.createTrajectoryPublisher("camera_trajectory", "world");
	pubMapPoints = rosConn.createPointCloudPublisher("map_points", "world");
	pubLocalMapPoints = rosConn.createPointCloudPublisher("local_map_points", "world");
	pubMask = rosConn.createImagePublisher("mask");
	cout << "Almost\n";
}

void run()
{

}

static void dumpMap(const shared_ptr<openvslam::publish::map_publisher> mapPub,
		pcl::PointCloud<pcl::PointXYZ> &mapCloud,
		pcl::PointCloud<pcl::PointXYZ> &localMapCloud,
		vector<Pose> &track)
{
	track.clear();
	mapCloud.clear();
	localMapCloud.clear();

	std::vector<openvslam::data::keyframe*> keyfrms;
	mapPub->get_keyframes(keyfrms);

	for (auto kf: keyfrms) {
		Pose pkf = kf->get_cam_pose_inv();
//		pkf = rot180*pkf; pkf = pkf*rot180x;
		track.push_back(pkf);
	}

	std::vector<openvslam::data::landmark*> landmarks;
	std::set<openvslam::data::landmark*> local_landmarks;
	mapPub->get_landmarks(landmarks, local_landmarks);

	for (auto &lm: landmarks) {
		auto pt = lm->get_pos_in_world();
		pcl::PointXYZ pt3d;
		pt3d.x = pt.x();
		pt3d.y = pt.y();
		pt3d.z = pt.z();
		mapCloud.push_back(pt3d);
	}
//	pcl::transformPointCloud(mapCloud, mapCloud, rot180.matrix().cast<float>());

	for (auto &lm: local_landmarks) {
		auto pt = lm->get_pos_in_world();
		pcl::PointXYZ pt3d;
		pt3d.x = pt.x();
		pt3d.y = pt.y();
		pt3d.z = pt.z();
		localMapCloud.push_back(pt3d);
	}
//	pcl::transformPointCloud(localMapCloud, localMapCloud, rot180.matrix().cast<float>());
}

static void dumpTrajectoryToFile(const vector<TTransform> &tfList, const string &filename="")
{
	fstream trackFd;
	ostream xo(NULL);

	if (filename.empty())
		xo.rdbuf(cout.rdbuf());
	else {
		trackFd.open(filename, ios_base::out|ios_base::trunc);
		if (!trackFd.is_open())
			throw runtime_error("Unable to create "+filename);
		xo.rdbuf(trackFd.rdbuf());
	}

	for (const auto &tf: tfList) {
		xo << tf.dump() << endl;
	}

	if (filename.empty()==false)
		trackFd.close();

	return;
}

void publishMap(const ros::Time &timestamp)
{
	pcl::PointCloud<pcl::PointXYZ> mapCloud, localMapCloud;
	vector<Pose> mapTrajectory;

	dumpMap(mapPub, mapCloud, localMapCloud, mapTrajectory);

	for (auto &ps: mapTrajectory) {
		ps = rot180*ps; ps = ps*rot180x;
	}
	pcl::transformPointCloud(mapCloud, mapCloud, rot180.matrix().cast<float>());
	pcl::transformPointCloud(localMapCloud, localMapCloud, rot180.matrix().cast<float>());

	rosConn.publishTrajectory(mapTrajectory, timestamp);
	rosConn.publishPointCloud(mapCloud, timestamp, pubMapPoints);
	rosConn.publishPointCloud(localMapCloud, timestamp, pubLocalMapPoints);
}

void publishFrame(const ros::Time &timestamp)
{
	cv::Mat frame = framePub->draw_frame();
	rosConn.publishImage(frame, pubOVFrame, timestamp);
	rosConn.publishImage(frameMask, pubMask, timestamp);

	Eigen::Matrix4d posee = mapPub->get_current_cam_pose().inverse();
	Pose pose = posee;

	if (!pose.isIdentity()) {
		// against origin point
		pose = rot180*pose;
		// in-place
		pose = pose*rot180x;
	}

	rosConn.publishPose(pubFramePose, pose, timestamp);

	PoseStamped poseWts(pose, timestamp.toBoost());
	frameTrajectory.push_back(poseWts);
}

void publish(const ros::Time &timestamp)
{
	ros::Time rt;
	if (useRealtime==false)
		rt = timestamp;
	else rt = ros::Time::now();
	publishFrame(rt);
	publishMap(rt);
}

bool dumpTrajectory(const std::string &f) const
{
	return frameTrajectory.dump(f);
}


bool useRealtime = false;
cv::Mat frameMask;
cv::Mat originalFrameImage;

private:
	openvslam::system &slam;
	const shared_ptr<openvslam::publish::frame_publisher> framePub;
	const shared_ptr<openvslam::publish::map_publisher> mapPub;
	Vmml::Mapper::ROSConnector rosConn;
	CameraPinholeParams camera;

	PubId
		pubOVFrame, pubMask,
		pubFramePose,
		pubCamTrack,
		pubMapPoints,
		pubLocalMapPoints;

	Trajectory frameTrajectory;
};


/////////////////

int main(int argc, char *argv[])
{
	Vmml::Mapper::ProgramOptions vsoProg;
	vsoProg.addSimpleOptions("vocabulary", "Path to vocabulary", true);
	vsoProg.addSimpleOptions<float>("start-time", "Mapping will start from x seconds");
	vsoProg.addSimpleOptions<float>("stop-time", "Maximum seconds from start");
	vsoProg.addSimpleOptions<float>("resample", "Reduce image rate to x Hz");
	vsoProg.addSimpleOptions<bool>("ros-time", "Use ROS time for published frames and map instead of bag time");
	vsoProg.parseCommandLineArgs(argc, argv);

	auto imageBag = vsoProg.getImageBag();
	auto lidarBag = vsoProg.getLidarScanBag();
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

	// GNSS
/*
	auto &mybag = vsoProg.getInputBag();
	auto trackGnss = TrajectoryGNSS::fromRosBag(mybag, vsoProg.getGnssTopic());
	trackGnss.dump((vsoProg.getWorkDir()/"gnss.csv").string());
	exit(2);
*/

	// Slam dunk
	openvslam::system SlamDunk (slamConfig, vocPath);
	PrimitiveViewer rosConn(vsoProg, SlamDunk);
	SlamDunk.startup();
	cout << "Ready\n";

	// Install our own signal interrupt handler
	signal(SIGINT, handleInterruptSignal);

	for (auto &frameId: targetFrameId) {

		auto image = imageBag->at(frameId);
		auto timestamp = imageBag->timeAt(frameId);
		cv::Mat mask;
		imagePipe.run(image, image, mask);
		rosConn.frameMask = mask;
		rosConn.originalFrameImage = image;

		SlamDunk.feed_monocular_frame(image, timestamp.toSec(), mask);
		rosConn.publish(timestamp);

		cout << "Frame#: " << frameId << endl;

		if (breakImageStream==true) {
			break;
		}
	}

	cout << "Termination\n";

	// Save map
	SlamDunk.save_map_database((vsoProg.getWorkDir()/"/test.map").string());
	pcl::PointCloud<pcl::PointXYZ> mapCloud, localMapCloud;
	vector<Pose> mapTrajectory;
	PrimitiveViewer::dumpMap(SlamDunk.get_map_publisher(), mapCloud, localMapCloud, mapTrajectory);
	rosConn.dumpTrajectory((vsoProg.getWorkDir()/"trajectory-slam.csv").string());
	pcl::io::savePCDFile((vsoProg.getWorkDir()/"vslam-map.pcd").string(), mapCloud, true);
	cout << "Data saved\n";

	// shutdown() crashes the program
	SlamDunk.shutdown();

	return 0;
}
