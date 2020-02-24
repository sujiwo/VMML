/*
 * Segmentation.cpp
 *
 *  Created on: Feb 22, 2020
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include <Segmentation.h>


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
}


cv::Mat
Segmentation::segment(const cv::Mat &sourceImage)
{
	auto input_layer = mNet->input_blobs()[0];
	input_layer->Reshape(1, numChannels, imgInputSize.height, imgInputSize.width);
	mNet->Reshape();

	/* Wrap the input layer of the network in separate cv::Mat objects
	 * (one per channel). This way we save one memcpy operation and we
	 * don't need to rely on cudaMemcpy2D. The last preprocessing
	 * operation will write the separate channels directly to the input
	 * layer. */
	vector<cv::Mat> input_channels;
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

	mNet->Forward();

	auto output_layer = mNet->output_blobs()[0];
	cv::Mat merged_output_image (output_layer->height(), output_layer->width(), CV_32F, const_cast<float *>(output_layer->cpu_data()));
	merged_output_image.convertTo(merged_output_image, CV_8U);
	return merged_output_image;
}


cv::Mat
Segmentation::buildMask(const cv::Mat &sourceImage)
{
	cv::Mat segmentedImg = segment(sourceImage);
	// Apply LUT
}


Segmentation::~Segmentation()
{}

} /* namespace Mapper */
} /* namespace Vmml */
