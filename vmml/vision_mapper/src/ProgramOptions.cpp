/*
 * ProgramOptions.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <exception>
#include <algorithm>
#include <fstream>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/convenience.hpp>
#include "ProgramOptions.h"
#include "INIReader.h"


namespace Vmml {
namespace Mapper {

using namespace std;
namespace po=boost::program_options;
using po::value;

/*
 * Common program options among mapper programs (without dashes) :
 * feature-mask
 * light-mask
 * bag-file
 * work-dir
 * resize
 * image-topic
 */

/*
 * These parameters will be read from config files:
 * camera parameters
 * transformation from lidar to camera
 */

const string dConfigFile = "config.ini";


#if CV_MAJOR_VERSION<=3
std::ostream& operator<< (std::ostream& out, const cv::Size& sz)
{
	out << '(' << sz.width << 'x' << sz.height << ')';
	return out;
}


std::ostream& operator<< (std::ostream& out, const cv::MatSize& sz)
{
	out << sz();
	return out;
}
#endif


ProgramOptions::ProgramOptions() :
	_options("Option Parser for mapper"),
	_vmPackagePath(boost::filesystem::path(ros::package::getPath("vision_mapper")))
{
	_options.add_options()
		("help", "Show help")

		("work-dir", 		value<string>()->notifier(bind(&ProgramOptions::openWorkDir, this, _1)), "Working directory")

		("feature-mask",	value<string>()->notifier(
				[&](const string &s){featureMaskImagePath = s;}),
				"Image file mask for feature detection")

		("light-mask", 		value<string>()->notifier(
				[&](const string &s){lightMaskImagePath = s;}),
				"Image file mask for gamma equalization")
	;

	addSimpleOptions("bag-file", "Bag file input", &inputBagPath);
	addSimpleOptions("resize", "Resize all masks and images from bags with a scalar value (0-1.0)", &imageResizeFactor);
	addSimpleOptions("image-topic", "Image topic to be used", &imageTopic);
	addSimpleOptions("lidar-topic", "Point cloud topic contained in bag", &lidarTopic);
	addSimpleOptions("gnss-topic", "GNSS topic contained in bag", &gnssTopic);
	addSimpleOptions("segnet-model", "Path to SegNet Model", &segnetModelPath);
	addSimpleOptions("segnet-weight", "Path to SegNet Weights", &segnetWeightsPath);
	addSimpleOptions("retinex", "Enable/disable Retinex enhancement", &useRetinex);
}


void ProgramOptions::showHelp()
{
	// XXX: fix me
	cout << _options << endl;
	exit(1);
}


ProgramOptions::~ProgramOptions()
{}


void
ProgramOptions::parseCommandLineArgs(int argc, char *argv[])
{
	rArgc = argc;
	rArgv = argv;

	vector<string> optionKeys;
	for (auto &opt: _options.options()) {
		optionKeys.push_back(opt->key(""));
	}

	try {
		po::store(po::parse_command_line(argc, argv, _options), _optionValues);
		po::notify(_optionValues);
	} catch (po::error &e) {
		cerr << "Parameter error: " << e.what() << endl;
		showHelp();
	}

	if (_optionValues.count("help")) showHelp();

	openInputs();
}


void
ProgramOptions::openWorkDir(const std::string &f)
{
	if (f.empty()) {
		workDir = boost::filesystem::current_path();
	}
	else workDir = Path(f);

	if (boost::filesystem::is_directory(workDir)==false)
		throw runtime_error(f+ " is not directory");

	auto configPath = workDir / dConfigFile;

	INIReader cfg(configPath.string());
	if (cfg.ParseError()!=0)
		throw runtime_error("Unable to open configuration file config.ini");

	camera0.fx = cfg.GetReal("camera_parameter", "fx", 0);
	camera0.fy = cfg.GetReal("camera_parameter", "fy", 0);
	camera0.cx = cfg.GetReal("camera_parameter", "cx", 0);
	camera0.cy = cfg.GetReal("camera_parameter", "cy", 0);
	camera0.distortionCoeffs[0] = cfg.GetReal("camera_parameter", "d1", 0);
	camera0.distortionCoeffs[1] = cfg.GetReal("camera_parameter", "d2", 0);
	camera0.distortionCoeffs[2] = cfg.GetReal("camera_parameter", "d3", 0);
	camera0.distortionCoeffs[3] = cfg.GetReal("camera_parameter", "d4", 0);
	camera0.distortionCoeffs[4] = cfg.GetReal("camera_parameter", "d5", 0);

	double tx = cfg.GetReal("lidar_to_camera", "x", 0),
		ty = cfg.GetReal("lidar_to_camera", "y", 0),
		tz = cfg.GetReal("lidar_to_camera", "z", 0),
		rx = cfg.GetReal("lidar_to_camera", "roll", 0),
		ry = cfg.GetReal("lidar_to_camera", "pitch", 0),
		rz = cfg.GetReal("lidar_to_camera", "yaw", 0);
	lidarToCamera = TTransform::from_XYZ_RPY(Eigen::Vector3d(tx,ty,tz), rx, ry, rz);

	if (featureMaskImagePath.empty())
		featureMaskImagePath = workDir / "feature_mask.png";
	if (lightMaskImagePath.empty())
		lightMaskImagePath = workDir / "light_mask.png";

	// GNSS Offset
	Eigen::Vector3d gov;
	gov.x() = cfg.GetReal("gnss_offset", "x", 0);
	gov.y() = cfg.GetReal("gnss_offset", "y", 0);
	gov.z() = cfg.GetReal("gnss_offset", "z", 0);
	double
		goRoll = cfg.GetReal("gnss_offset", "roll", 0),
		goPitch = cfg.GetReal("gnss_offset", "pitch", 0),
		goYaw = cfg.GetReal("gnss_offset", "yaw", 0);
	gnssOffset = Vmml::TTransform::from_XYZ_RPY(gov, goRoll, goPitch, goYaw);

	maxOrbKeypoints = cfg.GetInteger("ORB", "max_keypoints", 1000);
}


ImageBag::Ptr
ProgramOptions::getImageBag()
{
	if (imageBag==nullptr) {

		if (imageTopic.empty())
			throw runtime_error("Image topic is not set using --image-topic");

		imageBag = ImageBag::Ptr(new ImageBag(inputBag, imageTopic));
		cout << "Using `" << imageTopic << "' as image topic\n";
	}

	return imageBag;
}


LidarScanBag::Ptr
ProgramOptions::getLidarScanBag()
{
	if (lidarBag==nullptr) {

		if (lidarTopic.empty())
			throw runtime_error("Image topic is not set using --lidar-topic");
		lidarBag = LidarScanBag::Ptr(new LidarScanBag(inputBag, lidarTopic));
	}

	return lidarBag;
}


void
ProgramOptions::openInputs()
{
	// Set image pipeline
	if (boost::filesystem::exists(featureMaskImagePath))
		featureMask = cv::imread(featureMaskImagePath.string(), cv::IMREAD_GRAYSCALE);
	if (boost::filesystem::exists(lightMaskImagePath))
		lightMask = cv::imread(lightMaskImagePath.string(), cv::IMREAD_GRAYSCALE);

	imagePipeline.setResizeFactor(imageResizeFactor);
	if (segnetModelPath.empty()==false and segnetWeightsPath.empty()==false)
		imagePipeline.setSemanticSegmentation(segnetModelPath, segnetWeightsPath);
	imagePipeline.setFixedFeatureMask(featureMask);
	if (useRetinex)
		imagePipeline.setRetinex();

	if (inputBagPath.empty()==true)
		return;

	// Open Bag
	cout << "Opening bag... ";
	inputBag.open(inputBagPath.string(), rosbag::bagmode::Read);

	if (inputBag.isOpen()==true) {
		cout << "Done\n";
		auto bagTopics = RandomAccessBag::getTopicList(inputBag);
		vector<string> topicList;
		for (auto &tp: bagTopics) {
			cout << tp.first << "->" << tp.second << endl;
			topicList.push_back(tp.first);

			// Guess image topic
			if ((tp.second=="sensor_msgs/Image" or tp.second=="sensor_msgs/CompressedImage") and imageTopic.empty())
				imageTopic = tp.first;

			// Guess lidar topic
			if (tp.second=="sensor_msgs/PointCloud2" and lidarTopic.empty())
				lidarTopic = tp.first;

			// Guess GNSS topic
			if (gnssTopic.empty()==true) {
				if (tp.second=="nmea_msgs/Sentence" or tp.second=="sensor_msgs/NavSatFix")
					gnssTopic = tp.first;
			}
/*
			if (tp.second=="velodyne_msgs/VelodyneScan" and lidarTopic.empty())
				lidarTopic = tp.first;
*/
		}
	}

	else throw runtime_error("Unable to open bag file");

	getImageBag();
	cv::Size imageSize0 = imageBag->getImageDimensions();
	camera0.height = imageSize0.height;
	camera0.width = imageSize0.width;

	if (!lidarTopic.empty())
		getLidarScanBag();

	imagePipeline.setIntendedInputSize(imageSize0);
}

template<>
void Vmml::Mapper::ProgramOptions::addSimpleOptions(const std::string &opt, const std::string &description, bool* target, bool isRequired)
{
	auto boolSwitch = boost::program_options::bool_switch()->default_value(false);
	if (isRequired)
		boolSwitch->required();
	if (target!=nullptr)
		boolSwitch->notifier([target](const bool &v) {
		*target = v;
	});
	_options.add_options()(opt.c_str(), boolSwitch, description.c_str());
/*

	_options.add_options()(opt.c_str(), boost::program_options::bool_switch()
		->default_value(false)->notifier([target](const bool &v) {
			*target = v;
	}),
	description.c_str());
*/
}

template<>
void Vmml::Mapper::ProgramOptions::addSimpleOptions(const std::string &opt, const std::string &description, Vmml::Path *S, bool isRequired)
{
	auto value_segm = boost::program_options::value<string>();
	if (isRequired)
		value_segm->required();
	if (S!=nullptr)
		value_segm->notifier([S](const string &v){
			*S = v;
		});

	_options.add_options()(opt.c_str(), value_segm, description.c_str()
	);

}

} /* namespace Mapper */
} /* namespace Vmml */
