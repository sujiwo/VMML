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
#include <sensor_msgs/Image.h>
#include "RandomAccessBag.h"
#include "ImagePreprocessor.h"
#include "utilities.h"


namespace Vmml
{

class ImageBag : public RandomAccessBag
{
public:
	ImageBag(const rosbag::Bag &bag, const std::string &imageTopic);
	virtual ~ImageBag();

	cv::Mat at(unsigned int position, bool raw=false);
	cv::Mat at(const ptime &t, double *timeDiff=nullptr);
	cv::Mat at(const ros::Time &t, double *timeDiff=nullptr);

	cv::Mat getGrayscale(unsigned int position);

	bool save(unsigned int position, const std::string &filename, bool raw=false);

	typedef std::shared_ptr<ImageBag> Ptr;

	void getImageDimensions(uint &width, uint &height);

	cv::Size getImageDimensions();

	// Simulate reduced number of images per seconds
//	void desample(const float hz, std::vector<uint64> &desamplePos) const;

	sensor_msgs::ImageConstPtr getMessage(uint position);

protected:
	bool isCompressed=false;
};

} /* namespace Vmml */

#endif /* VMML_CORE_IMAGEBAG_H_ */
