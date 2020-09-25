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
#include "RandomAccessBag.h"


using namespace std;
namespace po=boost::program_options;


class IBoW_Mapper_App
{
public:

static const int defaultNumOfFeatures = 3000;

IBoW_Mapper_App(int argc, char *argv[])
{
	auto options = prepare_options();
	options.parseCommandLineArgs(argc, argv);

	// XXX: Modify this to use option parser
	featureDetector = cv::ORB::create(3000);
	bagFd.open(argv[2], rosbag::BagMode::Read);

	auto vTopicList = RandomAccessBag::getTopicList(bagFd);
	imageBag.reset(new RandomAccessBag(bagFd, argv[1]));

	imageBag->desample(7.5, messageList);
}

void processFrame(const uint bagMessageNumber)
{
}

void run()
{
}

static
PrgOpt::ProgramOption
prepare_options()
{
	PrgOpt::ProgramOption opts;
	opts.addSimpleOptions("bagfile", "Path to bagfile to be read");
	opts.addSimpleOptions("numfeats", string("Number of features from single image; default is "+to_string(defaultNumOfFeatures)));
	opts.addSimpleOptions("desample", "Reduce sample frequency of the bag");
	opts.addSimpleOptions("topic", "Image topic from bag");
	return opts;
}

private:
	cv::Ptr<cv::Feature2D> featureDetector;
	rosbag::Bag bagFd;
	RandomAccessBag::Ptr imageBag;
	string outputMapFilename;
	RandomAccessBag::DesampledMessageList messageList;

	PlaceRecognizer::IncrementalBoW mapperProc;
};


int main(int argc, char *argv[])
{
	IBoW_Mapper_App mapper(argc, argv);
	mapper.run();

	return 0;
}
