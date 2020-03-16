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
	tp get(const std::string &cf)
	{ return _optionValues.at(cf).as<tp>(); }

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
	 * Add an option. When user sets it, its value will be stored to S
	 * XXX: Add special value for bool
	 */
	template<typename T>
	void addSimpleOptions(const std::string &opt, const std::string &description, T& S)
	{
		_options.add_options()(opt.c_str(), boost::program_options::value<T>()
			->notifier([&](const T&v){
				S = v;
			}),
			description.c_str()
		);
	}

/*
	template<>
	void addSimpleOptions<bool>(const std::string &opt, const std::string &description, bool& target);
*/

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

/*
	void openFeatureMask(const std::string &f);
	void openLightMask(const std::string &f);
*/

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

	ImagePipeline imagePipeline;
};

} /* namespace Mapper */
} /* namespace Vmml */


template<>
void Vmml::Mapper::ProgramOptions::addSimpleOptions(const std::string &opt, const std::string &description, bool& target);


#endif /* VMML_MAPPER_PROGRAMOPTIONS_H_ */
