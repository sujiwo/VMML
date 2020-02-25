/*
 * Segmentation.cpp
 *
 *  Created on: Feb 22, 2020
 *      Author: sujiwo
 */

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "vmml/utilities.h"
#include "Segmentation.h"


using namespace caffe;
using namespace std;


namespace Vmml {
namespace Mapper {

Segmentation::Segmentation(const std::string &modelPath, const std::string &weights)
{
	Caffe::set_mode(Caffe::GPU);

	mNet.reset( new caffe::Net<float> (modelPath, caffe::TEST) );
	mNet->CopyTrainedLayersFrom(weights);

	auto input_layer = mNet->input_blobs()[0];
	imgInputSize = cv::Size(input_layer->width(), input_layer->height());
	numChannels = input_layer->channels();
	cout << "Input size: " << imgInputSize << endl;
}


cv::Mat
Segmentation::segment(const cv::Mat &sourceImage)
{
	/*
	 * For multi-threaded applications (ex: ROS nodes),
	 * we need to set mode to GPU again
	 */
	Caffe::set_mode(Caffe::GPU);

	auto origin_size=sourceImage.size();

	auto input_layer = mNet->input_blobs()[0];
	input_layer->Reshape(1, numChannels, imgInputSize.height, imgInputSize.width);
	mNet->Reshape();

	/* Wrap the input layer of the network in separate cv::Mat objects
	 * (one per channel). This way we save one memcpy operation and we
	 * don't need to rely on cudaMemcpy2D. The last preprocessing
	 * operation will write the separate channels directly to the input
	 * layer. */
	vector<cv::Mat> input_channels;
	input_layer = mNet->input_blobs()[0];
	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for (int i = 0; i < input_layer->channels(); ++i) {
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels.push_back(channel);
		input_data += width * height;
	}

	// Preprocess
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample;
	if (sourceImage.channels() == 3 && numChannels == 1)
		cv::cvtColor(sourceImage, sample, cv::COLOR_BGR2GRAY);
	else if (sourceImage.channels() == 4 && numChannels == 1)
		cv::cvtColor(sourceImage, sample, cv::COLOR_BGRA2GRAY);
	else if (sourceImage.channels() == 4 && numChannels == 3)
		cv::cvtColor(sourceImage, sample, cv::COLOR_BGRA2BGR);
	else if (sourceImage.channels() == 1 && numChannels == 3)
		cv::cvtColor(sourceImage, sample, cv::COLOR_GRAY2BGR);
	else
		sample = sourceImage;

	cv::Mat sample_resized;
	if (sample.size() != imgInputSize)
		cv::resize(sample, sample_resized, imgInputSize);
	else
		sample_resized = sample;

	cv::Mat sample_float;
	if (numChannels == 3)
		sample_resized.convertTo(sample_float, CV_32FC3);
	else
		sample_resized.convertTo(sample_float, CV_32FC1);
	cv::split(sample_float, input_channels);

	auto t1=getCurrentTime();
	mNet->Forward();
	auto t2=getCurrentTime();
	cout << "Forward time: " << toSeconds(t2-t1) << " seconds" << endl;

	auto output_layer = mNet->output_blobs()[0];
	cv::Mat merged_output_image (output_layer->height(), output_layer->width(), CV_32F, const_cast<float *>(output_layer->cpu_data())),
		merged_gray;
	merged_output_image.convertTo(merged_gray, CV_8U);
	cv::resize(merged_gray, merged_gray, origin_size, 0, 0, cv::INTER_NEAREST);
	return merged_gray;
}



const uint8_t _defaultSegmentationMask[] = {
	0x0,	//	    0:'Sky',
	0xff,	//	    1:'Building',
	0xff,	//	    2:'Pole',
	0xff,	//	    3:'Road Marking',
	0xff,	//	    4:'Road',
	0xff,	//	    5:'Pavement',
	0x0,	//	    6:'Tree',
	0xff,	//	    7:'Sign Symbol',
	0xff,	//	    8:'Fence',
	0x0,	//	    9:'Vehicle',
	0x0,	//	    10:'Pedestrian',
	0x0		//	    11:'Bike'
};

const cv::Mat defaultSegmentationMask(cv::Size(1,12), CV_8UC1, const_cast<uint8_t*>(_defaultSegmentationMask));


cv::Mat
Segmentation::buildMask(const cv::Mat &sourceImage)
{
	cv::Mat segmentedImg = segment(sourceImage);
	// Apply LUT
	cv::Mat mask(segmentedImg.size(), CV_8UC1);
	for (int r=0; r<mask.rows; ++r) {
		for (int c=0; c<mask.cols; ++c) {
			mask.at<uchar>(r,c) = _defaultSegmentationMask[segmentedImg.at<uchar>(r,c)];
		}
	}
	return mask;
}


Segmentation::~Segmentation()
{}

} /* namespace Mapper */
} /* namespace Vmml */
