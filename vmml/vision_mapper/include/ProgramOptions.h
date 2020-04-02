/*
 * ProgramOptions.h
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_PROGRAMOPTIONS_H_
#define VMML_MAPPER_PROGRAMOPTIONS_H_

#include <memory>
#include <opencv2/core.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include "vmml/utilities.h"
#include "vmml/CameraPinholeParams.h"
#include "vmml/Pose.h"
#include "vmml/ImageBag.h"
#include "vmml/LidarScanBag.h"
#include "ImagePipeline.h"


namespace Vmml {
namespace Mapper {

class ProgramOptions {
public:
	ProgramOptions();
	virtual ~ProgramOptions();

	void parseCommandLineArgs(int argc, char *argv[]);

	cv::Mat& getFeatureMask()
	{ return featureMask; }

	cv::Mat& getLightMask()
	{ return lightMask; }

	rosbag::Bag& getInputBag()
	{ return inputBag; }

	ImageBag::Ptr getImageBag();

	LidarScanBag::Ptr getLidarScanBag();

	const CameraPinholeParams& getOriginalCameraParameters() const
	{ return camera0; }

	const TTransform& getLidarToCameraTransform() const
	{ return lidarToCamera; }

	template<typename tp>
	tp get(const std::string &cf, const tp &defaultValue)
	{
		try {
			return _optionValues.at(cf).as<tp>();
		} catch (std::out_of_range &e) { return defaultValue; }
	}

	const double getImageResizeFactor() const
	{ return imageResizeFactor; }

	const std::string& getImageTopic() const
	{ return imageTopic; }

	const std::string& getLidarTopic() const
	{ return lidarTopic; }

	const Path& getWorkDir() const
	{ return workDir; }

	const Path& getBagPath() const
	{ return inputBagPath; }

	inline boost::program_options::options_description_easy_init addOptions()
	{ return _options.add_options(); }


	/*
	 * Add an option. When user sets it, optionally its value will be stored to S
	 */
	template<typename T=std::string>
	void addSimpleOptions(const std::string &opt, const std::string &description, T* S=nullptr, bool isRequired=false)
	{
		auto value_segm = boost::program_options::value<T>();
		if (isRequired)
			value_segm->required();
		if (S!=nullptr)
			value_segm->notifier([S](const T &v){
				*S = v;
			});
		else
			value_segm->notifier([](const T&v){});

		_options.add_options()(opt.c_str(), value_segm, description.c_str()
		);
	}

	void addSimpleOptions(const std::string &opt, const std::string &description, bool isRequired=false)
	{
		return addSimpleOptions<std::string>(opt, description, nullptr, isRequired);
	}

//	void addSimpleOptions(bool isRequired, const std::string &opt, const std::string &Description);

	inline int getArgc() const
	{ return rArgc; }

	inline char** getArgv() const
	{ return rArgv; }

	template<typename T>
	const T& getOptionValue(const std::string &key)
	{ return _optionValues.at(key).as<T>(); }

	ImagePipeline& getImagePipeline()
	{ return imagePipeline; }

	inline Vmml::CameraPinholeParams
	getWorkingCameraParameter() const
	{ return camera0 * imageResizeFactor; }

	inline Vmml::TTransform getGnssOffset() const
	{ return gnssOffset; }

	uint getMaxOrbKeypoints() const
	{ return maxOrbKeypoints; }

protected:
	boost::program_options::options_description _options;
	boost::program_options::variables_map _optionValues;
	Path _vmPackagePath;

	// Raw arguments
	int rArgc;
	char **rArgv;

	void openInputs();

	void setupImagePipeline();

	void showHelp();

	void openWorkDir(const std::string &);

	// Common input for mapper programs
	cv::Mat
		featureMask,
		lightMask;
	double imageResizeFactor=1.0;

	std::string
		imageTopic,
		lidarTopic,
		gnssTopic;

	// Segnet-related
	std::string
		segnetModelPath,
		segnetWeightsPath;

	Path workDir;

	Path inputBagPath;
	Path featureMaskImagePath, lightMaskImagePath;
	rosbag::Bag inputBag;
	ImageBag::Ptr imageBag=nullptr;
	LidarScanBag::Ptr lidarBag=nullptr;

	CameraPinholeParams camera0;

	TTransform lidarToCamera=TTransform::Identity();

	TTransform gnssOffset = TTransform::Identity();

	ImagePipeline imagePipeline;
	bool useRetinex;

	uint maxOrbKeypoints=10;
};

} /* namespace Mapper */
} /* namespace Vmml */


template<>
void Vmml::Mapper::ProgramOptions::addSimpleOptions(const std::string &opt, const std::string &description, bool *target, bool isRequired);

template<>
void Vmml::Mapper::ProgramOptions::addSimpleOptions(const std::string &opt, const std::string &description, Vmml::Path *target, bool isRequired);


#endif /* VMML_MAPPER_PROGRAMOPTIONS_H_ */
