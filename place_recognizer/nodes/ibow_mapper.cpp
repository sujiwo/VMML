/*
 * Mapper node for creating visual maps of places
 */

#include <string>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <boost/program_options.hpp>
#include "IncrementalBoW.h"
#include "ProgramOptionParser.h"
#include "ImageBag.h"


using namespace std;
namespace po=boost::program_options;


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

	imageBag->desample(options.get<float>("desample", defaultSampleImageRate), messageList);
}

void run()
{
	for (auto mId: messageList) {
		auto frameImg = imageBag->at(mId);

		std::vector<cv::KeyPoint> kpList;
		cv::Mat descriptors;
		featureDetector->detectAndCompute(frameImg, cv::Mat(), kpList, descriptors, false);

		if (mId==messageList.front()) {

		}
		else {

		}
	}
}

static
PrgOpt::ProgramOption
prepare_options()
{
	PrgOpt::ProgramOption opts;
	opts.addSimpleOptions("bagfile", "Path to bagfile to be read");
	opts.addSimpleOptions("numfeats", string("Number of features from single image; default is "+to_string(defaultNumOfFeatures)));
	opts.addSimpleOptions("desample", "Reduce sample frequency of the bag; default is "+to_string(defaultSampleImageRate));
	opts.addSimpleOptions("topic", "Image topic from bag");
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
};


int main(int argc, char *argv[])
{
	IBoW_Mapper_App mapper(argc, argv);
	mapper.run();

	return 0;
}
