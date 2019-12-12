/*
 * vocabulary_creator.cpp
 *
 *  Created on: Dec 6, 2019
 *      Author: sujiwo
 */

#include <vector>
#include <string>
#include <iostream>
#include <rosbag/bag.h>
#include "ImageBag.h"
#include "VisionMap.h"
#include "Trajectory.h"
#include "TrajectoryGNSS.h"
#include "Matcher.h"
#include "ORBVocabulary.h"
#include "utilities.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;


// 5 cm/s
const float
linearVelocityThreshold = 0.05,
linearDistThreshold = 100.0,
linearIdleThreshold = 10.0,
imageSameThrScore = 0.15;

const uint
numberOfFeatures = 3000;


cv::Ptr<cv::DescriptorMatcher> bMatcher = cv::BFMatcher::create();
cv::Ptr<cv::FeatureDetector> bFeats = cv::ORB::create(
		numberOfFeatures,
		1.2,
		8,
		32,
		0,
		2,
		cv::ORB::HARRIS_SCORE,
		32,
		10);


float compareAndScore(const BaseFrame &f1, const BaseFrame &f2)
{
	Matcher::PairList fr12Matches;
	Matcher::matchEpipolar(f1, f2, fr12Matches, bMatcher);
	return (float)fr12Matches.size() / (float)f1.numOfKeyPoints();
}


int main(int argc, char *argv[])
{
	Path mybagPath(argv[1]);
	rosbag::Bag mybag(mybagPath.string());

	ImageBag images(mybag, "/camera1/image_raw", 0.5);
	uint width, height;
	images.getImageDimensions(width, height);

	CameraPinholeParams camera0(0, 0, 0, 0, width, height);

	auto trackGnss = TrajectoryGNSS::fromRosBag(mybag, "/nmea_sentence");
	trackGnss.dump("gnss.csv");

	Mapper::RVizConnector rosConn(argc, argv, "vocabulary_creator");

	Trajectory trackImage;
	// Image database
	ORBVocabulary myVocab;
	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;

	auto imageAnchor = BaseFrame::create(images.at(0), camera0);
	imageAnchor->computeFeatures(bFeats);

	const int maxLim = 5000;
//	const int maxLim = images.size();
	for (int i=1; i<maxLim; ++i) {
		auto curImage = BaseFrame::create(images.at(i), camera0);
		ptime imageTimestamp = images.timeAt(i).toBoost();
		curImage->computeFeatures(bFeats);

		if (curImage->numOfKeyPoints()<=10) {
			cout << "XXX!\n";
			string si=to_string(i)+".png";
			cv::imwrite(si, curImage->getImage());
			continue;
		}

		float comparisonScore = compareAndScore(*imageAnchor, *curImage);
		bool isKeyFrame=false;

		if (comparisonScore<=imageSameThrScore) {
			imageAnchor = curImage;
			rosConn.publishBaseFrame(*curImage);
			PoseStamped imagePose = trackGnss.at(imageTimestamp);
			trackImage.push_back(imagePose);
			isKeyFrame = true;

			vector<cv::Mat> kfDescriptor = curImage->getDescriptorVector();
			if (kfDescriptor.size()!=0) {
				keymapFeatures.push_back(kfDescriptor);
			}
		}

		cout << i+1 << " / " << maxLim << (isKeyFrame==true?"*":"") << "; Score: " << comparisonScore << endl;
	}

	trackGnss.dump("gnss.csv");
	trackImage.dump("images.csv");

	cout << "Creating vocabulary for " << keymapFeatures.size() << " keyframes... " << flush;
	myVocab.create(keymapFeatures);
	myVocab.saveToTextFile("vocabulary.txt");
	cout << "Done\n";

	return 0;
}
