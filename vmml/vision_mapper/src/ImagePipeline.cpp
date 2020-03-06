/*
 * ImagePipeline.cpp
 *
 *  Created on: Feb 26, 2020
 *      Author: sujiwo
 */

#include <exception>
#include <thread>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "vmml/ImagePreprocessor.h"
#include "vmml/Retinex.h"
#include "ImagePipeline.h"

using namespace std;


namespace Vmml {
namespace Mapper {


ImagePipeline::ImagePipeline(const cv::Size& inputSize) :
	intentInputSize(inputSize),
	outputSize(cv::Size(640,480))
{
}


ImagePipeline::~ImagePipeline()
{
}


void
ImagePipeline::setResizeFactor(const float f)
{
	resizeFactor=f;
	outputSize = cv::Size(intentInputSize.width*f, intentInputSize.height*f);
	setFixedFeatureMask(stdMask);
}


void
ImagePipeline::setFixedFeatureMask(const string &imageMaskPath)
{
	cv::Mat mmask = cv::imread(imageMaskPath, cv::IMREAD_GRAYSCALE);
	if (stdMask.empty()==true)
		throw runtime_error("Unable to open image ");
	return setFixedFeatureMask(mmask);
}


void
ImagePipeline::setFixedFeatureMask(const cv::Mat &fmask)
{
	stdMask = fmask.clone();
	if (resizeFactor!=1.0 and stdMask.empty()==false)
		cv::resize(stdMask, stdMaskResized, cv::Size(), resizeFactor, resizeFactor);
}


void
ImagePipeline::setSemanticSegmentation(const std::string &modelPath, const std::string &weights)
{
	gSegment.reset(new Vmml::Mapper::Segmentation(modelPath, weights));
}


void
ImagePipeline::run(const cv::Mat &imageRgb, cv::Mat &imageOut, cv::Mat &mask)
{
	cv::Mat imageInput;

	if (resizeFactor!=1.0)
		cv::resize(imageRgb, imageInput, cv::Size(), resizeFactor, resizeFactor);
	else
		imageInput = imageRgb;

	thread imageBrightnessThread ([&,this](){
		if (retinexPrc!=nullptr)
			imageOut = retinexPrc->run(imageInput);
		else
			imageOut = ImagePreprocessor::autoAdjustGammaRGB(imageInput, gammaMeteringMask);
	});

	thread semanticSegmentThread ([&,this](){
		if (gSegment==NULL)
			mask = stdMaskResized.clone();
		else {
			cv::Mat ssMask = gSegment->buildMask(imageInput);
			if (stdMaskResized.empty()==false)
				mask = stdMaskResized & ssMask;
			else mask = ssMask;
			cv::resize(mask, mask, imageInput.size(), 0, 0, cv::INTER_NEAREST);
		}
	});

	imageBrightnessThread.join();
	semanticSegmentThread.join();
}


void
ImagePipeline::runRaw(const cv::Mat &imageRawSource, cv::Mat &imageOut, cv::Mat &mask)
{
	// XXX: Need to research all options for demosaicing algorithms
	// See https://docs.opencv.org/master/d8/d01/group__imgproc__color__conversions.html
	cv::Mat imageRgb;
	cv::demosaicing(imageRawSource, imageRgb, cv::COLOR_BayerBG2BGR_EA);
	return run(imageRgb, imageOut, mask);
}


void
ImagePipeline::run(const sensor_msgs::Image &imageBg, cv::Mat &imageOut, cv::Mat &mask)
{
	auto imageRgb = cv_bridge::toCvCopy(imageBg, "bgr8");
	return run(imageRgb->image, imageOut, mask);
}


void
ImagePipeline::setRetinex(
	const float _ss[3],
	const float _lowClip,
	const float _highClip)
{
	retinexPrc.reset(new Retinex(_ss, _lowClip, _highClip));
}


} /* namespace Mapper */
} /* namespace Vmml */
