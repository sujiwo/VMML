/*
 * ProgramOptions.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */
#include <string>
#include <iostream>
#include <exception>
#include <algorithm>
#include <fstream>
#include <ros/package.h>
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


ProgramOptions::ProgramOptions() :
	_options("Option Parser for mapper"),
	_vmPackagePath(boost::filesystem::path(ros::package::getPath("vision_mapper")))
{
	_options.add_options()
		("help", value<string>()->notifier(boost::bind(&ProgramOptions::showHelp, this, _1)), "Show help")

		("feature-mask", 	value<string>()->notifier(bind(&ProgramOptions::openFeatureMask, this, _1)), "Image file mask for feature detection")

		("light-mask", 		value<string>(),						"Image file mask for gamma equalization")

		("bag-file", 		value<string>()->notifier(bind(&ProgramOptions::openBag, this, _1)), "Bag file input")

		("work-dir", 		value<string>()->notifier(bind(&ProgramOptions::openWorkDir, this, _1)), "Working directory")

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
}


void
ProgramOptions::openFeatureMask(const std::string &f)
{
	featureMask = cv::imread(f, cv::IMREAD_GRAYSCALE);
	auto factor = _optionValues["resize"].as<double>();
	cout << "Mask size: " << featureMask.size << endl;
	if (factor!=1.0) {
		cv::resize(featureMask, featureMask, cv::Size(), factor, factor);
		cout << "Resized to " << featureMask.size << endl;
	}
}


void
ProgramOptions::openLightMask(const std::string &f)
{
	lightMask = cv::imread(f, cv::IMREAD_GRAYSCALE);
	auto factor = _optionValues["resize"].as<double>();
	cout << "Light Mask size: " << lightMask.size << endl;
	if (factor!=1.0) {
		cv::resize(lightMask, lightMask, cv::Size(), factor, factor);
		cout << "Resized to " << lightMask.size << endl;
	}
}


void
ProgramOptions::openBag(const std::string &f)
{
	cout << "Opening bag... ";
	inputBag.open(f, rosbag::bagmode::Read);

	if (inputBag.isOpen()==true) {
		cout << "Done\n";
		auto bagTopics = RandomAccessBag::getTopicList(inputBag);
		vector<string> topicList;
		for (auto &tp: bagTopics) {
			cout << tp.first << "->" << tp.second << endl;
			topicList.push_back(tp.first);
		}
/*

		auto it=_optionValues["image-topic"].as<string>();
		if (std::find(topicList.begin(), topicList.end(), it)==topicList.end())
			cout << "Image topic not found: " << it << endl;

		it=_optionValues["image-lidar"].as<string>();
		if (std::find(topicList.begin(), topicList.end(), it)==topicList.end())
			cout << "Lidar topic not found: " << it << endl;

		it=_optionValues["gnss-topic"].as<string>();
		if (std::find(topicList.begin(), topicList.end(), it)==topicList.end())
			cout << "GNSS topic not found: " << it << endl;
*/
	}

	else cout << "Failed\n";
}


void
ProgramOptions::openWorkDir(const std::string &f)
{
	workDir = Path(f);
	if (boost::filesystem::is_directory(workDir)==false)
		throw runtime_error(f+ " is not directory");

	auto configPath = workDir / dConfigFile;

	INIReader cfg(configPath.string());
/*

	fstream cfgFd (configPath.string(), fstream::in);
	if (!cfgFd.is_open())
		throw runtime_error("Unable to open "+configPath.string());

	// fetch more program options from here
	po::store(po::parse_config_file(cfgFd, _options, true), _optionValues);


	cfgFd.close();
*/
}


void
ProgramOptions::setValues()
{

}

} /* namespace Mapper */
} /* namespace Vmml */
