
#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImageDatabase.h"
#include "vmml/BaseFrame.h"
#include "ImagePipeline.h"
#include "ProgramOptions.h"
#include "yaml-cpp/yaml.h"
#include "vision_mapper/place_recognizer.h"

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/tracking_module.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"


using namespace std;
using namespace Vmml::Mapper;
using Vmml::Pose;
using VFrame=openvslam::data::frame;


YAML::Node createDummyConfig(const Vmml::Mapper::ProgramOptions &po)
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
	vsConf["Camera.fps"] = 10.0;
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


class VoslPlaceRecognizerService
{
public:
	VoslPlaceRecognizerService(int argc, char *argv[])
	{
		progOptions.addSimpleOptions("vocabulary", "Path to vocabulary", true);
		progOptions.addSimpleOptions("map-file", "Path to map file", true);
		progOptions.parseCommandLineArgs(argc, argv);

		imagePipe = &progOptions.getImagePipeline();

		ros::init(argc, argv, "vosl_place_recognizer");
		rosNode.reset(new ros::NodeHandle);
		placeRecognSrv = rosNode->advertiseService("place_recognizer", &VoslPlaceRecognizerService::service, this);

		// Prepare SLAM
		auto vocPath = progOptions.get<string>("vocabulary", "");
		auto dummyYaml = createDummyConfig(progOptions);
		auto slamConfig = make_shared<openvslam::config>(dummyYaml);

		placerc.reset(new openvslam::system(slamConfig, vocPath));
		placerc->load_map_database(progOptions.get<string>("map-file", ""));
		placerc->startup();
		frameDb = placerc->_getFrameDatabase();
		placerc->disable_mapping_module();

		// Dump information
		auto keyframes = placerc->_getMapDatabase()->get_all_keyframes();
		std::sort(keyframes.begin(), keyframes.end(),
			[&](const openvslam::data::keyframe *k1, const openvslam::data::keyframe *k2)
			{ return k1->id_ < k2->id_; }
		);
		saveTrajectory("vosl-kftrajectory.csv", keyframes);

		cout << "Ready\n";
	}

	~VoslPlaceRecognizerService()
	{}

	void run()
	{
		ros::spin();
	}

	bool service(
		vision_mapper::place_recognizer::Request &request,
		vision_mapper::place_recognizer::Response &response)
	{
		cv::Mat workImg, mask;
		imagePipe->run(request.input, workImg, mask);

		VFrame curfrm(workImg, 0,
			placerc->_getTracker()->_getMonocularFeatureExtractor(),
			placerc->_getVocabulary(),
			placerc->_getCamera(),
			100.0,
			mask);
		auto kfCandidates = frameDb->acquire_relocalization_candidates(&curfrm);

		if (kfCandidates.empty()==true)
			return false;

		for (const auto kf: kfCandidates) {
			response.keyframeId.push_back(kf->id_);
		}
		return true;
	}

protected:
	Vmml::Mapper::ProgramOptions progOptions;
	shared_ptr<ros::NodeHandle> rosNode;
	ros::ServiceServer placeRecognSrv;
	ImagePipeline *imagePipe;

	shared_ptr<openvslam::system> placerc;
	openvslam::data::bow_database *frameDb;

	void saveTrajectory(const string &path, const vector<openvslam::data::keyframe*> &kfList)
	{
		fstream dumpKfFd(path, ios::out);

		for (const auto kf: kfList) {
			Pose pkf = kf->get_cam_pose().inverse();
			dumpKfFd << kf->id_ << ' ';
			dumpKfFd << Vmml::dumpVector(pkf.position()) << endl;
		}
	}
};


int main(int argc, char *argv[])
{
	VoslPlaceRecognizerService server(argc, argv);
	exit(1);
	server.run();
	return 0;
}
