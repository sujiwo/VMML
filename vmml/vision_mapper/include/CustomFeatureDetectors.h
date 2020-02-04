/*
 * CustomFeatureDetectors.h
 *
 *  Created on: Jan 30, 2020
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_CUSTOMFEATUREDETECTORS_H_
#define VMML_MAPPER_CUSTOMFEATUREDETECTORS_H_


#include <opencv2/features2d.hpp>


namespace Vmml {
namespace Mapper {

class CustomFeatureDetectors : public cv::FeatureDetector
{
public:
	CustomFeatureDetectors();
	virtual ~CustomFeatureDetectors();
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_CUSTOMFEATUREDETECTORS_H_ */
