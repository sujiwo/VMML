/*
 * ImagePipeline.h
 *
 *  Created on: Feb 26, 2020
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_IMAGEPIPELINE_H_
#define VMML_MAPPER_IMAGEPIPELINE_H_


/*
 * Integrates all image-processing-related routines
 * prior to feature detection and indexing/triangulation.
 *
 * Currently limited to gamma enhancement and semantic segmentation
 */

#include <memory>
#include <string>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include "vmml/Retinex.h"
#include "Segmentation.h"


namespace Vmml {
namespace Mapper {

const float retinexSigmaDefault[] = { 15, 80, 250 };


class ImagePipeline {
public:
	ImagePipeline();

	/*
	 * Setup functions
	 */
	void setFixedFeatureMask(const std::string &imageMaskPath);

	void setFixedFeatureMask(const cv::Mat &fmask);

	inline void setResizeFactor(const float f)
	{ resizeFactor=f; }

	inline float getResizeFactor() const
	{ return resizeFactor; }

	const cv::Size getOutputSize() const
	{ return outputSize; }

	void setOutputSize(const cv::Size &sz)
	{ outputSize = sz; }

	// XXX: Unstable
	void setRetinex(
		const float _ss[3] = retinexSigmaDefault,
		const float _lowClip = 0.01,
		const float _highClip = 0.999);

	void setSemanticSegmentation(const std::string &modelPath, const std::string &weights);

	virtual ~ImagePipeline();

	void run(const cv::Mat &imageRgbSource, cv::Mat &imageOut, cv::Mat &mask);

	void run(const sensor_msgs::Image &imageBg, cv::Mat &imageOut, cv::Mat &mask);

	inline void run(const sensor_msgs::Image::ConstPtr &imageBg, cv::Mat &imageOut, cv::Mat &mask)
	{ run(*imageBg, imageOut, mask); }

protected:

	std::shared_ptr<Segmentation> gSegment=NULL;

	cv::Mat stdMask;

	float resizeFactor=1.0;

	cv::Size outputSize;

	std::shared_ptr<Retinex> retinexPrc;
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_IMAGEPIPELINE_H_ */