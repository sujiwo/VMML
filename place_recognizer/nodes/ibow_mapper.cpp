/*
 * Mapper node for creating visual maps of places
 */

#include <string>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "IncrementalBoW.h"
#include "ProgramOptionParser.h"
#include "ImageBag.h"
#include "npy.hpp"


using namespace std;
namespace po=boost::program_options;
using Path = boost::filesystem::path;


class IBoW_Mapper_App
{
public:

static const int defaultNumOfFeatures = 3000;
static constexpr float defaultSampleImageRate = 7.5;

IBoW_Mapper_App(int argc, char *argv[])
{
	auto options = prepare_options();
	options.parseCommandLineArgs(argc, argv);

	auto numFeats = options.get<int>("numfeats", defaultNumOfFeatures);
	featureDetector = cv::ORB::create(numFeats);

	bagFd.open(options.get<string>("bagfile", ""), rosbag::BagMode::Read);

	auto imageTopic = options.get<string>("topic", "");
	if (imageTopic.empty())
		imageTopic = PlaceRecognizer::ImageBag::suggestTopic(bagFd);
	imageBag.reset(new PlaceRecognizer::ImageBag(bagFd, imageTopic));
	cout << "Using `" << imageBag->getTopic() << "' as image topic" << endl;

	startTimeSeconds = options.get<float>("start-time", startTimeSeconds);
	maxSecondsFromStart = options.get<float>("stop-time", maxSecondsFromStart);
	imageBag->setTimeConstraint(startTimeSeconds, maxSecondsFromStart);
	imageBag->desample(options.get<float>("desample", defaultSampleImageRate), messageList);
	cout << "# of target frames: " << imageBag->size() << endl;

	// Dump message IDs by using Numpy
	npy::saveMat(messageList, "/tmp/debugmapper.log");

	auto _mapOutputPath = options.get<string>("mapfile", "");
	if (_mapOutputPath.empty()) {
		//
	}
	else mapOutputPath = Path(_mapOutputPath);
	return;
}

void run()
{
	for (auto mId: messageList) {
		auto frameImg = imageBag->at(mId);

		std::vector<cv::KeyPoint> kpList;
		cv::Mat descriptors;
		featureDetector->detectAndCompute(frameImg, cv::Mat(), kpList, descriptors, false);

		if (mId==messageList.front()) {
			mapperProc.addImage(mId, kpList, descriptors);
		}
		else {
			mapperProc.addImage2(mId, kpList, descriptors);
		}

		cout << mId << "/" << imageBag->size() << endl;
	}

	cout << "Saving to " << mapOutputPath.string() << "... ";
	mapperProc.saveToDisk(mapOutputPath.string());
	cout << "Done\n";

//	mapperProc.
}

static
PrgOpt::ProgramOption
prepare_options()
{
	PrgOpt::ProgramOption opts;

	// XXX: put bagfile as positional argument
	opts.addSimpleOptions("bagfile", "Path to bagfile to be read");
	opts.addSimpleOptions<int>
		("numfeats", string("Number of features from single image; default is "+to_string(defaultNumOfFeatures)));
	opts.addSimpleOptions<float>
		("desample", "Reduce sample frequency of the bag; default is "+to_string(defaultSampleImageRate));
	opts.addSimpleOptions("topic", "Image topic from bag");
	opts.addSimpleOptions
		<decltype(IBoW_Mapper_App::startTimeSeconds)>
		("start-time", "Seconds from start of bag time");
	opts.addSimpleOptions
		<decltype(IBoW_Mapper_App::startTimeSeconds)>
		("stop-time", "Maximum seconds from start");
	opts.addSimpleOptions("mapfile", "Map file output path");
	return opts;
}

private:
	cv::Ptr<cv::Feature2D> featureDetector;
	rosbag::Bag bagFd;
	PlaceRecognizer::ImageBag::Ptr imageBag;
	string outputMapFilename;
	RandomAccessBag::DesampledMessageList messageList;
	bool isCompressedImage=false;
	PlaceRecognizer::IncrementalBoW mapperProc;

	// Time constraint for bag
	float startTimeSeconds=0,
		maxSecondsFromStart=-1;

	Path mapOutputPath;
};


int main(int argc, char *argv[])
{
	IBoW_Mapper_App mapper(argc, argv);
	mapper.run();

	return 0;
}
