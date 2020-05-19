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

class SIFT : public cv::FeatureDetector
{
public:
	SIFT();
	virtual ~SIFT();
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_CUSTOMFEATUREDETECTORS_H_ */
