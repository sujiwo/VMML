/*
 * index_mapper.cpp
 *
 *  Created on: Jun 2, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <exception>
#include <ros/package.h>

#include "OxfordDataset.h"
#include "ROSConnector.h"
#include "ProgramOptions.h"


using Vmml::Mapper::ROSConnector;
using Vmml::Mapper::ProgramOptions;
using namespace oxf;
using namespace std;


int main (int argc, char *argv[])
{
	ProgramOptions progOpts;
	Vmml::Path dataSrcPath, mapTargetPath;
	progOpts.addSimpleOptions("data-dir", "Directory of dataset", &dataSrcPath, true);
	progOpts.addSimpleOptions("map-file", "Filename of map file to save to", &mapTargetPath, true);

	progOpts.parseCommandLineArgs(argc, argv);

	// Set-up image pipelines and feature extractor
	Vmml::Path packg (ros::package::getPath("oxford_test"));
	auto &imagePipe = progOpts.getImagePipeline();
	imagePipe.setFixedFeatureMask((packg/"model/oxford_mask.png").string());
	auto featureDetector = cv::ORB::create(
		6000,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
		10);

	OxfordDataset dataSrc(dataSrcPath.string());
	auto targetFrames = dataSrc.desample(7.0);

	ROSConnector rosPub(argc, argv, "oxford_index_mapper");
	auto pubId = rosPub.createImagePublisher("oxford", Vmml::CameraPinholeParams(), "center");

	for (int i=0; i<targetFrames.size(); ++i) {

		auto oxRecord = dataSrc.at(targetFrames[i]);
		cv::cvtColor(oxRecord.center_image, oxRecord.center_image, cv::COLOR_RGB2BGR);

		cv::Mat mask, imageReady;
		imagePipe.run(oxRecord.center_image, imageReady, mask);

		rosPub.publishImage(imageReady, pubId, ros::Time::fromBoost(oxRecord.timestamp));
		cout << targetFrames[i] << endl;
	}

	return 0;
}
