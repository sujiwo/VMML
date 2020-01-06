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


ProgramOptions::ProgramOptions() :
	_options("Option Parser for mapper"),
	_vmPackagePath(boost::filesystem::path(ros::package::getPath("vision_mapper")))
{
	_options.add_options()
		("help", value<string>()->notifier(boost::bind(&ProgramOptions::showHelp, this, _1)), "Show help")

		("work-dir", 		value<string>()->default_value("")->notifier(bind(&ProgramOptions::openWorkDir, this, _1)), "Working directory")

		("feature-mask",	value<string>()->notifier(
				[&](const string &s){featureMaskImagePath = s;}),
				"Image file mask for feature detection")

		("light-mask", 		value<string>()->notifier(
				[&](const string &s){lightMaskImagePath = s;}),
				"Image file mask for gamma equalization")

		("bag-file", 		value<string>()->notifier(
				[&](const string &b){inputBagPath = b;}),
				"Bag file input")

		("resize", 			value<double>()->default_value(1.0)->notifier(
				[&](const double &d){imageResizeFactor=d;}),
				"Resize all masks and images from bags with a scalar value (0-1.0)")

		("image-topic",		value<string>()->default_value("")->notifier(
				[&](const string &t){imageTopic=t;}),
				"Image topic contained in bag")

		("lidar-topic",		value<string>()->default_value("")->notifier(
				[&](const string &t){lidarTopic=t;}),
				"Point cloud topic contained in bag")

		("gnss-topic",		value<string>()->default_value("")->notifier(
				[&](const string &t){gnssTopic=t;}),
				"GNSS topic contained in bag")
	;
}


void ProgramOptions::showHelp(const string &s)
{
	// XXX: fix me
	cout << "This is help" << endl;
}


ProgramOptions::~ProgramOptions()
{}


void
ProgramOptions::parseCommandLineArgs(int argc, char *argv[])
{
	po::store(po::parse_command_line(argc, argv, _options), _optionValues);
	po::notify(_optionValues);

	openInputs();
}


void
ProgramOptions::openFeatureMask(const std::string &f)
{
	featureMask = cv::imread(f, cv::IMREAD_GRAYSCALE);

	if (featureMask.empty())
		throw runtime_error("Unable to open feature mask file");

	auto factor = _optionValues["resize"].as<double>();
	cout << "Mask size: " << featureMask.size << endl;
	if (factor!=1.0) {
		cv::resize(featureMask, featureMask, cv::Size(), factor, factor);
		cout << "Resized to " << featureMask.size << endl;
	}
	camera0.mask = featureMask;
}


void
ProgramOptions::openLightMask(const std::string &f)
{
	lightMask = cv::imread(f, cv::IMREAD_GRAYSCALE);

	if (lightMask.empty())
		return;

	auto factor = _optionValues["resize"].as<double>();
	cout << "Light Mask size: " << lightMask.size << endl;
	if (factor!=1.0) {
		cv::resize(lightMask, lightMask, cv::Size(), factor, factor);
		cout << "Resized to " << lightMask.size << endl;
	}
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
}


ImageBag::Ptr
ProgramOptions::getImageBag()
{
	if (imageBag==nullptr) {

		if (imageTopic.empty())
			throw runtime_error("Image topic is not set using --image-topic");

		imageBag = ImageBag::Ptr(new ImageBag(inputBag, imageTopic, imageResizeFactor));
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
			if (tp.second=="sensor_msgs/Image" and imageTopic.empty())
				imageTopic = tp.first;

			// Guess lidar topic
			if (tp.second=="sensor_msgs/PointCloud2" and lidarTopic.empty())
				lidarTopic = tp.first;

/*
			if (tp.second=="velodyne_msgs/VelodyneScan" and lidarTopic.empty())
				lidarTopic = tp.first;
*/
		}
	}

	else cout << "Failed\n";

	getImageBag();
	auto imgSz = imageBag->at(0);
	camera0.height = imgSz.rows * 1/imageResizeFactor;
	camera0.width = imgSz.cols * 1/imageResizeFactor;
	camera0 = camera0 * imageResizeFactor;

	if (!lidarTopic.empty())
		getLidarScanBag();

	// masks will be set with size corrected by resize parameter
	openFeatureMask(featureMaskImagePath.string());
	if (boost::filesystem::exists(lightMaskImagePath))
		imageBag->setGammaMeteringMask(lightMaskImagePath.string());
}

} /* namespace Mapper */
} /* namespace Vmml */
