/*
 * ProgramOptions.h
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_PROGRAMOPTIONS_H_
#define VMML_MAPPER_PROGRAMOPTIONS_H_

#include <opencv2/core.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "vmml/utilities.h"
#include "vmml/CameraPinholeParams.h"
#include "vmml/Pose.h"
#include "vmml/ImageBag.h"
#include "vmml/LidarScanBag.h"


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

	const CameraPinholeParams& getCameraParameters() const
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

protected:
	boost::program_options::options_description _options;
	boost::program_options::variables_map _optionValues;
	Path _vmPackagePath;

	void setValues();

	void showHelp(const std::string &h);

	void openFeatureMask(const std::string &f);
	void openLightMask(const std::string &f);
	void openBag(const std::string &);
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

	Path workDir;

	Path inputBagPath;
	rosbag::Bag inputBag;
	ImageBag::Ptr imageBag=nullptr;
	LidarScanBag::Ptr lidarBag=nullptr;

	CameraPinholeParams camera0;

	TTransform lidarToCamera=TTransform::Identity();
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_PROGRAMOPTIONS_H_ */
