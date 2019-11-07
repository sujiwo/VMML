/*
 * ImageBag.h
 *
 *  Created on: Oct 17, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_IMAGEBAG_H_
#define VMML_CORE_IMAGEBAG_H_

#include <string>
#include <opencv2/core.hpp>
#include "RandomAccessBag.h"
#include "utilities.h"


namespace Vmml
{

class ImageBag : public RandomAccessBag
{
public:
	ImageBag(const rosbag::Bag &bag, const std::string &imageTopic, float zoom=1.0);
	virtual ~ImageBag();

	cv::Mat at(unsigned int position);

	bool save(unsigned int position, const std::string &filename);

	typedef std::shared_ptr<ImageBag> Ptr;

protected:
	float zoomRatio;
};

} /* namespace Vmml */

#endif /* VMML_CORE_IMAGEBAG_H_ */
