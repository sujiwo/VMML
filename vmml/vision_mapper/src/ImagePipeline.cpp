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


ImagePipeline::ImagePipeline() :
	outputSize(cv::Size(640,480))
{
}


ImagePipeline::~ImagePipeline()
{
}


void
ImagePipeline::setFixedFeatureMask(const string &imageMaskPath)
{
	stdMask = cv::imread(imageMaskPath, cv::IMREAD_GRAYSCALE);
	if (stdMask.empty()==true)
		throw runtime_error("Unable to open image ");
}


void
ImagePipeline::setFixedFeatureMask(const cv::Mat &fmask)
{ stdMask = fmask.clone(); }


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
			imageOut = ImagePreprocessor::autoAdjustGammaRGB(imageInput);
	});

	thread semanticSegmentThread ([&,this](){
		if (gSegment==NULL)
			mask = stdMask.clone();
		else {
			cv::Mat ssMask = gSegment->buildMask(imageInput);
			if (stdMask.empty()==false)
				mask = stdMask & ssMask;
			else mask = ssMask;
			cv::resize(mask, mask, imageInput.size(), 0, 0, cv::INTER_NEAREST);
		}
	});

	imageBrightnessThread.join();
	semanticSegmentThread.join();
}


void
ImagePipeline::run(const sensor_msgs::Image &imageBg, cv::Mat &imageOut, cv::Mat &mask)
{
	// XXX: Need better demosaicing algorithm
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