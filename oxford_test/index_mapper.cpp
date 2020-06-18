/*
 * index_mapper.cpp,
 * adjusted for Oxford Dataset
 */

#include <opencv2/features2d.hpp>
#include <iostream>
#include <ROSConnector.h>
#include <ProgramOptions.h>
#include <vmml/ImageDatabase.h>
#include "OxfordDataset.h"

using namespace std;
using Vmml::Path;
using Vmml::ImageDatabase;
using oxf::OxfordDataset;


cv::Ptr<cv::FeatureDetector> createFeatureDetector(const Vmml::Mapper::ProgramOptions &opt)
{
	auto orb = cv::ORB::create(
			opt.getMaxOrbKeypoints(),
			1.2,
			8,
			31,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			31,
			10);
	cout << "ORB created with " << orb->getMaxFeatures() << " points" << endl;

	return orb;
}



int main(int argc, char *argv[])
{
	Vmml::Mapper::ROSConnector rosCon(argc, argv, "oxford_mapper", ros::init_options::NoSigintHandler);
	Vmml::Mapper::ProgramOptions progOpts;
	Path dataSrcDir;

	// Mapping options
	double offsetStart=0, offsetStop=-1;
	string mapFilename;
	progOpts.addSimpleOptions("offset-start", "Start of dataset, default is 0", &offsetStart);
	progOpts.addSimpleOptions("offset-stop", "Stop of dataset, default is end", &offsetStop);
	progOpts.addSimpleOptions("map-name", "Filename for result map, stored in work directory; default to basename of datasource", &mapFilename);

	progOpts.removeOptions("bag-file");
	progOpts.addSimpleOptions("data-dir", "Path to Oxford data source directory", &dataSrcDir, true);
	progOpts.parseCommandLineArgs(argc, argv);

	// Mapping setup begins
	Path targetMapfilename;
	if (mapFilename.empty())
		targetMapfilename = progOpts.getWorkDir() / (boost::filesystem::basename(dataSrcDir) + ".map");
	else targetMapfilename = progOpts.getWorkDir() / mapFilename;

	OxfordDataset dataSrc(dataSrcDir.string());
	auto sampleMaps = dataSrc.desample(6.0, offsetStart, offsetStop);

	auto &imagePipe = progOpts.getImagePipeline();
	auto featureDetector = createFeatureDetector(progOpts);
	// Mapping setup ends

	auto pubId = rosCon.createImagePublisher("oxford", "center");
	ImageDatabase imageDb;

	for (uint imageIdx=0; imageIdx<sampleMaps.size(); ++imageIdx) {

		auto sampleId = sampleMaps[imageIdx];
		auto record = dataSrc.at(sampleId);

		cv::Mat imageReady, mask;
		imagePipe.run(record.center_image, imageReady, mask);

		cv::Mat descriptors;
		vector<cv::KeyPoint> keypoints;
		featureDetector->detectAndCompute(imageReady, mask, keypoints, descriptors);

		if (imageIdx==0) {
			imageDb.addImage(imageIdx, keypoints, descriptors);
		}
		else {
			imageDb.addImage2(imageIdx, keypoints, descriptors);
		}

		imageDb.keyframeIdToBag[imageIdx] = sampleId;
		rosCon.publishImage(imageReady, pubId);

		cout << imageIdx+1 << " / " << sampleMaps.size() << endl;
	}

	cout << "Done mapping" << endl;
	imageDb.saveToDisk(targetMapfilename.string());
	cout << "Done saving to " << boost::filesystem::absolute(targetMapfilename).string() << endl;
	return 0;
}
