/*
 * preprocess_image_bag.cpp
 *
 *  Created on: Sep 8, 2020
 *      Author: sujiwo
 *
 *  This program preprocesses image stream from a Bag file using
 *  selected image contrast enhancement method,
 *  then record its results into a new bag.
 */


#include <iostream>
#include <omp.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include "ProgramOptions.h"
#include "RandomAccessBag.h"
#include "im_enhance.h"


using namespace std;
namespace fs=boost::filesystem;

const float alpha = 0.3975;


struct PreprocWorkers {
	cv::Mat image;
	std_msgs::Header header;
	ros::Time msgTime;
	sensor_msgs::CompressedImage imageOutputMsg;
};


int main(int argc, char *argv[])
{
	int ch;
	fs::path outputPath, inputPath;

	Vmml::Mapper::ProgramOptions po;
	po.clearOptions();
	po.addSimpleOptions("method", "Image preprocessing method (1:AGC, 2:MSRCP, 3:IlluminationInvariant, 4:ExpFuse, 5:DHE)", &ch, true);
	po.addSimpleOptions("input", "Input bag file name", &inputPath, true);
	po.addSimpleOptions("topic", "Image topic from input");
	po.addSimpleOptions("output", "Output bag file name", &outputPath, true);
	po.parseCommandLineArgs(argc, argv);

	rosbag::Bag inputBagFd(inputPath.string(), rosbag::bagmode::Read);

	string topic;
	auto allTopics = RandomAccessBag::getTopicList(inputBagFd);
	try {
		topic = po.getOptionValue<string>("topic");
		if (allTopics[topic]!="sensor_msgs/Image")
			throw out_of_range("");
	} catch (out_of_range &e) {
		for (auto tp: allTopics) {
			if (tp.second=="sensor_msgs/Image") {
				topic = tp.first;
				cout << "Using " << topic << " as input" << endl;
				break;
			}
		}
	}

	RandomAccessBag inputImageStream(inputBagFd, topic);
	cout << "Input length: " << inputImageStream.size() << endl;

	rosbag::Bag outputBagFd(outputPath.string(), rosbag::bagmode::Write);

	const int numcpu = omp_get_num_procs(),
			msgmax = 100,
			steps = int(ceil (msgmax / double(numcpu)));

	vector<PreprocWorkers> imageResults;
	imageResults.reserve(numcpu);

	for (auto iteration=0; iteration<steps; iteration++) {

		imageResults.clear();

		int max_par_msg = numcpu;
		if (iteration==steps-1)
			max_par_msg = (msgmax%numcpu==0 ? numcpu : msgmax%numcpu);

		// preload messages
		for (int p=0; p<max_par_msg; ++p) {
			PreprocWorkers w;
			auto t = iteration*numcpu + p;
			w.msgTime = inputImageStream.timeAt(t);
			auto imageInputMsg = inputImageStream.at<sensor_msgs::Image>(t);
			w.image = cv_bridge::toCvCopy(imageInputMsg, "bgr8")->image;
			w.header = imageInputMsg->header;
			imageResults.push_back(w);
		}

		cv_bridge::CvImage newImg;
		cv::Mat image, res;
#pragma omp parallel for private(image, res, newImg)
		for (int p=0; p<max_par_msg; ++p) {
			image = imageResults[p].image;

			// Do preprocess
			switch (ch) {
				case 1: res = ice::autoAdjustGammaRGB(image); break;
				case 2: res = ice::multiScaleRetinexCP(image); break;
				case 3: res = ice::toIlluminatiInvariant(image, alpha); break;
				case 4: res = ice::exposureFusion(image); break;
				case 5: res = ice::dynamicHistogramEqualization(image); break;
			}

			newImg.image = res;
			newImg.encoding = sensor_msgs::image_encodings::BGR8;
			newImg.header = imageResults[p].header;
			newImg.toCompressedImageMsg(imageResults[p].imageOutputMsg, cv_bridge::Format::PNG);
		}
#pragma omp barrier

		for (int p=0; p<max_par_msg; ++p)
			outputBagFd.write(topic, imageResults[p].msgTime, imageResults[p].imageOutputMsg);
	}

/*
	for (int p=0; p<100; ++p) {
		auto msg = inputImageStream.at<sensor_msgs::Image>(p);
		auto msgTime = inputImageStream.timeAt(p);
		cv::Mat image = cv_bridge::toCvShare( msg, "bgr8" )->image;

		// Do preprocess
		cv::Mat res;
		auto t1 = Vmml::getCurrentTime();
		switch (ch) {
			case 1: res = ice::autoAdjustGammaRGB(image); break;
			case 2: res = ice::multiScaleRetinexCP(image); break;
			case 3: res = ice::toIlluminatiInvariant(image, alpha); break;
			case 4: res = ice::exposureFusion(image); break;
			case 5: res = ice::dynamicHistogramEqualization(image); break;
		}
		auto t2 = Vmml::getCurrentTime();
		cout << p+1 << ": finished in " << Vmml::toSeconds(t2-t1) << "s" << endl;

		cv_bridge::CvImage newImg;
		newImg.image = res;
		newImg.encoding = sensor_msgs::image_encodings::BGR8;
		newImg.header = msg->header;

		outputBagFd.write(topic, msgTime, newImg.toCompressedImageMsg(cv_bridge::Format::PNG));
	}
*/

	return 0;
}
