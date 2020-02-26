/*
 * ImagePipeline.cpp
 *
 *  Created on: Feb 26, 2020
 *      Author: sujiwo
 */

#include <exception>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "vmml/ImagePreprocessor.h"
#include <cv_bridge/cv_bridge.h>
#include <ImagePipeline.h>

using namespace std;


namespace Vmml {
namespace Mapper {


ImagePipeline::ImagePipeline()
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
ImagePipeline::setSemanticSegmentation(const std::string &modelPath, const std::string &weights)
{
	gSegment.reset(new Vmml::Mapper::Segmentation(modelPath, weights));
}


void
ImagePipeline::run(const cv::Mat &imageRgb, cv::Mat &imageOut, cv::Mat &mask)
{
	imageOut = ImagePreprocessor::autoAdjustGammaRGB(imageRgb);

	if (gSegment==NULL)
		mask = stdMask.clone();
	else {
		cv::Mat ssMask = gSegment->buildMask(imageRgb);
		if (stdMask.empty()==false)
			mask = stdMask & ssMask;
		else mask = ssMask;
	}
}


void
ImagePipeline::run(const sensor_msgs::Image::ConstPtr &imageInBg, cv::Mat &imageOut, cv::Mat &mask)
{
	// XXX: Need better demosaicing algorithm
	auto imageRgb = cv_bridge::toCvCopy(imageInBg, "bgr8");
	return run(imageRgb->image, imageOut, mask);
}


} /* namespace Mapper */
} /* namespace Vmml */
