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
#include "ImagePreprocessor.h"
#include "utilities.h"


namespace Vmml
{

class ImageBag : public RandomAccessBag
{
public:
	ImageBag(const rosbag::Bag &bag, const std::string &imageTopic, float zoom=1.0);
	virtual ~ImageBag();

	cv::Mat at(unsigned int position, bool raw=false);
	cv::Mat at(const ptime &t);
	cv::Mat at(const ros::Time &t);

	cv::Mat getGrayscale(unsigned int position);

	bool save(unsigned int position, const std::string &filename, bool raw=false);

	void setGammaMeteringMask(const cv::Mat &meteringMask);
	void setGammaMeteringMask(const std::string &path=std::string());

	typedef std::shared_ptr<ImageBag> Ptr;

	const ImagePreprocessor& getPreprocessor() const
	{ return imgPreps; }

	void getImageDimensions(uint &width, uint &height);

	void getOriginalImageDimensions(uint &width, uint &height);

	// Simulate reduced number of images per seconds
	void desample(const float hz, std::vector<uint64> &desamplePos) const;


protected:
	float zoomRatio;

	cv::Mat equalizeGamma(const cv::Mat &src) const;

	ImagePreprocessor imgPreps;
};

} /* namespace Vmml */

#endif /* VMML_CORE_IMAGEBAG_H_ */
