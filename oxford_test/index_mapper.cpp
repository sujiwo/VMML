/*
 * index_mapper.cpp
 *
 *  Created on: Jun 2, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <exception>
#include <ros/package.h>

#include <vmml/ImageDatabase.h>
#include "OxfordDataset.h"
#include "ROSConnector.h"
#include "ProgramOptions.h"


using Vmml::Mapper::ROSConnector;
using Vmml::Mapper::ProgramOptions;
using namespace oxf;
using namespace std;


namespace oxf {

class IndexCreator
{
protected:
ROSConnector *rosCon;
Vmml::Mapper::ImagePipeline *imgPipe;
vector<uint> targetFrames;
Vmml::Path packg;
cv::Ptr<cv::Feature2D> featureDetector;
OxfordDataset *dataSrc;
ROSConnector::PublisherId imgPub;
Vmml::Trajectory vehicleTrack;

public:
	IndexCreator(ROSConnector &roscon_, Vmml::Mapper::ImagePipeline &imgPipe_, OxfordDataset &datasrc_):
		rosCon(&roscon_),
		imgPipe(&imgPipe_),
		packg(ros::package::getPath("oxford_test")),
		dataSrc(&datasrc_)
	{
		featureDetector = cv::ORB::create(
			3000,
			1.2,
			8,
			31,
			0,
			2,
			cv::ORB::HARRIS_SCORE,
			31,
			10);

		imgPipe->setFixedFeatureMask((packg/"model/oxford_mask.png").string());
		targetFrames = dataSrc->desample(7.0);

		imgPub = rosCon->createImagePublisher("oxford", dataSrc->getCameraParameters(), "center");

		vehicleTrack = dataSrc->getInsTrajectory();

		auto cam0 = dataSrc->getCameraParameters();
		imgPipe->setIntendedInputSize(cam0.getImageSize());
	}

	void run()
	{
		for (int i=0; i<targetFrames.size(); ++i) {

			cv::Mat mask, imageReady, imageBgr;
			auto oxRecord = dataSrc->at(targetFrames[i]);
	//		cv::cvtColor(oxRecord.center_image, imageBgr, cv::COLOR_RGB2BGR);

//			imgPipe.run(oxRecord.center_image, imageReady, mask);

			rosCon->publishImage(oxRecord.center_image, imgPub, ros::Time::fromBoost(oxRecord.timestamp));
			cout << targetFrames[i] << endl;
		}
	}

};

}	// namespace oxf


cv::Ptr<cv::Feature2D> featureDetector =
	cv::ORB::create(
		3000,
		1.2,
		8,
		31,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		31,
		10);



int main (int argc, char *argv[])
{
	ProgramOptions progOpts;
	Vmml::Path dataSrcPath, mapTargetPath;
	progOpts.addSimpleOptions("data-dir", "Directory of dataset", &dataSrcPath, true);
	progOpts.addSimpleOptions("map-file", "Filename of map file to save to", &mapTargetPath, true);

	progOpts.parseCommandLineArgs(argc, argv);

	OxfordDataset dataSrc(dataSrcPath.string());
	ROSConnector rosPub(argc, argv, "oxford_index_mapper");

//	XXX: passing reference/pointer to this class makes program crash. Why ?
/*
	IndexCreator indexImgCreator(rosPub, progOpts.getImagePipeline(), dataSrc);
	indexImgCreator.run();
*/

	auto pubImg = rosPub.createImagePublisher("oxford", Vmml::CameraPinholeParams(), "center");
	auto &pipeline = progOpts.getImagePipeline();

	Vmml::ImageDatabase imageDb;

	auto dsSamples = dataSrc.desample(7.0);

	uint keyframeId = 0;
	for (auto s: dsSamples) {

		auto record = dataSrc.at(s);
		cv::Mat imagePrep, imageMask;
		pipeline.run(record.center_image, imagePrep, imageMask);

		cv::Mat descriptors;
		vector<cv::KeyPoint> keypoints;
		featureDetector->detectAndCompute(imagePrep, imageMask, keypoints, descriptors);

		if (keyframeId==0) {
			// No checks
			imageDb.addImage(keyframeId, keypoints, descriptors);
		}
		else
			// Perform checks
			imageDb.addImage2(keyframeId, keypoints, descriptors);

		rosPub.publishImage(imagePrep, pubImg, ros::Time::now());
		cout << s << " / " << dataSrc.size() << endl;

		imageDb.keyframeIdToBag[keyframeId] = s;
		keyframeId+=1;
	}

	cout << "Done mapping\n";

	imageDb.saveToDisk(mapTargetPath.string());
	cout << "Saved to " << mapTargetPath.string() << endl;

	return 0;
}
