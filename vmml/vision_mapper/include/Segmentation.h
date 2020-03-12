/*
 * Segmentation.h
 *
 *  Created on: Feb 22, 2020
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_SEGMENTATION_H_
#define VMML_MAPPER_SEGMENTATION_H_


#include <string>
#include <memory>
#include <opencv2/core.hpp>
#define USE_OPENCV 1
#include <caffe/caffe.hpp>


namespace Vmml {
namespace Mapper {


class Segmentation
{
public:
	Segmentation(const std::string &modelPath, const std::string &weights);

	cv::Mat segment(const cv::Mat &sourceImage);

	cv::Mat buildMask(const cv::Mat &sourceImage);

	virtual ~Segmentation();

	/*
	 * Return last result
	 */
	inline cv::Mat getLastResult() const
	{ return lastSegnResult.clone(); }

protected:
	std::shared_ptr<caffe::Net<float>> mNet;
	cv::Size imgInputSize;
	uint numChannels;

	cv::Mat lastSegnResult;
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_SEGMENTATION_H_ */
