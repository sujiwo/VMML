/*
 * ImagePreprocessor.h
 *
 *  Created on: Nov 28, 2018
 *      Author: sujiwo
 */

#ifndef VMML_SRC_IMAGEPREPROCESSOR_H_
#define VMML_SRC_IMAGEPREPROCESSOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImagePreprocessor
{
public:

	enum ProcessMode {
		AS_IS = 0,
		AGC = 1,
		ILLUMINATI = 2
	};

	ImagePreprocessor();
	ImagePreprocessor(ProcessMode m);
	virtual ~ImagePreprocessor();

//	cv::Mat preprocess(const cv::Mat &src);
	void preprocess(cv::Mat &srcInplace) const;

	void setMask (const cv::Mat &maskSrc);

	inline void setIAlpha (const float &a)
	{ iAlpha = a; }

	inline void setMode (ProcessMode m)
	{ pMode = m; }

	static float detectSmear (cv::Mat &rgbImage, const float tolerance);
	static cv::Mat cdf (cv::Mat &grayImage, cv::Mat mask=cv::Mat());
	static cv::Mat setGamma (const cv::Mat &grayImage, const float gamma, bool LUT_only=false);
//	static cv::Mat autoAdjustGammaRGB (cv::Mat &rgbImage, cv::Mat mask=cv::Mat());
	static cv::Mat autoAdjustGammaRGB (const cv::Mat &rgbImage, const cv::Mat &mask=cv::Mat());
	static cv::Mat autoAdjustGammaMono (cv::Mat &grayImage, float *gamma=NULL, cv::Mat mask=cv::Mat());
	static cv::Mat toIlluminatiInvariant (const cv::Mat &bayerImage, const float alpha);

	/*
	 * Multi-scale Retinex with Color Restoration
	 */
	static cv::Mat retinaHdr(const cv::Mat &rgbImage,
		cv::Vec3f weights=cv::Vec3f(-1,-1,-1),
		cv::Vec3f sigmas=cv::Vec3f(),
		int gain=1,
		int offset=0,
		double restoration_factor=1.0,
		double color_gain=1.0);

	static cv::Mat GrayWorld(const cv::Mat &rgbImage);

	static cv::Mat histogram (cv::Mat &inputMono, cv::Mat mask=cv::Mat());

	inline bool maskIsEmpty() const
	{ return mask.empty(); }

protected:

	ProcessMode pMode;
	cv::Mat mask;

	float iAlpha;

	// Color image buffer
	std::vector<cv::Mat> rgbImageBuf;
};


cv::Mat histogram (cv::Mat &inputMono, cv::Mat mask=cv::Mat());

#endif /* VMML_SRC_IMAGEPREPROCESSOR_H_ */
