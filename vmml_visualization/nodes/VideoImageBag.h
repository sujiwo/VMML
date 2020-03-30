/*
 * VideoImageBag.h
 *
 *  Created on: Mar 30, 2020
 *      Author: sujiwo
 */

#ifndef VMML_VIDEOIMAGEBAG_H_
#define VMML_VIDEOIMAGEBAG_H_

#include <memory>
#include <string>
#include "opencv2/videoio.hpp"
#include "vmml/ImageBag.h"


namespace Vmml {

class VideoImageBag : public ImageBag
{
public:

	VideoImageBag(const std::string &sourcePath);

	virtual ~VideoImageBag();

protected:
	std::shared_ptr<cv::VideoCapture> sink;
};

} /* namespace Vmml */

#endif /* VMML_VIDEOIMAGEBAG_H_ */
