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


namespace oxf {

class IndexCreator
{
protected:
ROSConnector &rosCon;
Vmml::Mapper::ImagePipeline &imgPipe;
vector<uint> targetFrames;
Vmml::Path packg;
cv::Ptr<cv::Feature2D> featureDetector;
OxfordDataset &dataSrc;
ROSConnector::PublisherId imgPub;
Vmml::Trajectory vehicleTrack;

public:
	IndexCreator(ROSConnector &roscon_, Vmml::Mapper::ImagePipeline &imgPipe_, OxfordDataset &datasrc_):
		rosCon(roscon_),
		imgPipe(imgPipe_),
		packg(ros::package::getPath("oxford_test")),
		dataSrc(datasrc_)
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

		imgPipe.setFixedFeatureMask((packg/"model/oxford_mask.png").string());
		targetFrames = dataSrc.desample(7.0);

		imgPub = rosCon.createImagePublisher("oxford", dataSrc.getCameraParameters(), "center");

		vehicleTrack = dataSrc.getInsTrajectory();
	}

	void run()
	{
		for (int i=0; i<targetFrames.size(); ++i) {

			cv::Mat mask, imageReady, imageBgr;
			auto oxRecord = dataSrc.at(targetFrames[i]);
	//		cv::cvtColor(oxRecord.center_image, imageBgr, cv::COLOR_RGB2BGR);

//			imgPipe.run(oxRecord.center_image, imageReady, mask);

			rosCon.publishImage(oxRecord.center_image, imgPub, ros::Time::fromBoost(oxRecord.timestamp));
			cout << targetFrames[i] << endl;
		}
	}

};

}	// namespace oxf



int main (int argc, char *argv[])
{
	ProgramOptions progOpts;
	Vmml::Path dataSrcPath, mapTargetPath;
	progOpts.addSimpleOptions("data-dir", "Directory of dataset", &dataSrcPath, true);
	progOpts.addSimpleOptions("map-file", "Filename of map file to save to", &mapTargetPath, true);

	progOpts.parseCommandLineArgs(argc, argv);

	OxfordDataset dataSrc(dataSrcPath.string());
	ROSConnector rosPub(argc, argv, "oxford_index_mapper");

	IndexCreator indexImgCreator(rosPub, progOpts.getImagePipeline(), dataSrc);
	indexImgCreator.run();

	return 0;
}
