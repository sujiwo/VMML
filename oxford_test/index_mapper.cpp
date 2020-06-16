/*
 * index_mapper.cpp,
 * adjusted for Oxford Dataset
 */

#include <iostream>
#include <ROSConnector.h>
#include <ProgramOptions.h>
#include "OxfordDataset.h"

using namespace std;
using Vmml::Path;
using oxf::OxfordDataset;


int main(int argc, char *argv[])
{
	Vmml::Mapper::ROSConnector rosCon(argc, argv, "oxford_mapper", ros::init_options::NoSigintHandler);
	Vmml::Mapper::ProgramOptions progOpts;
	Path dataSrcDir;

	progOpts.removeOptions("bag-file");
	progOpts.addSimpleOptions("data-dir", "Path to Oxford data source directory", &dataSrcDir, true);
	progOpts.parseCommandLineArgs(argc, argv);

	OxfordDataset dataSrc(dataSrcDir.string());
	auto sampleMaps = dataSrc.desample(6.0);

	auto &imagePipe = progOpts.getImagePipeline();

	auto pubId = rosCon.createImagePublisher("oxford", "center");

	for (uint s: sampleMaps) {
		auto record = dataSrc.at(s);

		cv::Mat imageReady, mask;
		imagePipe.run(record.center_image, imageReady, mask);

		rosCon.publishImage(imageReady, pubId);
	}

	cout << "Done" << endl;
	return 0;
}
