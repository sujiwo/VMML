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

protected:
	std::shared_ptr<caffe::Net<float>> mNet;
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_SEGMENTATION_H_ */
