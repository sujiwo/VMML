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
	cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create(
			opt.getMaxOrbKeypoints(),
			1.2,
			8,
			31,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			31,
			10);

	return orb;
}



int main(int argc, char *argv[])
{
	Vmml::Mapper::ROSConnector rosCon(argc, argv, "oxford_mapper", ros::init_options::NoSigintHandler);
	Vmml::Mapper::ProgramOptions progOpts;
	Path dataSrcDir;

	progOpts.removeOptions("bag-file");
	progOpts.addSimpleOptions("data-dir", "Path to Oxford data source directory", &dataSrcDir, true);
	progOpts.parseCommandLineArgs(argc, argv);

	auto targetMapfilename = progOpts.getWorkDir() / (boost::filesystem::basename(dataSrcDir) + ".map");

	OxfordDataset dataSrc(dataSrcDir.string());
	auto sampleMaps = dataSrc.desample(6.0);

	auto &imagePipe = progOpts.getImagePipeline();
	auto featureDetector = createFeatureDetector(progOpts);

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
	}

	cout << "Done mapping" << endl;
	imageDb.saveToDisk(targetMapfilename.string());
	cout << "Done saving to " << targetMapfilename.string() << endl;
	return 0;
}
