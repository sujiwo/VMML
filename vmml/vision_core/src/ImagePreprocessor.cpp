/*
 * ImagePreprocessor.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: sujiwo
 */

#include <vector>
#include "vmml/ImagePreprocessor.h"


using namespace std;

ImagePreprocessor::ImagePreprocessor() :
	pMode(ProcessMode::AS_IS),
	rgbImageBuf(3)
{}

ImagePreprocessor::ImagePreprocessor(ProcessMode m) :
	pMode(m),
	rgbImageBuf(3)
{}

ImagePreprocessor::~ImagePreprocessor()
{
	// TODO Auto-generated destructor stub
}

void ImagePreprocessor::setMask (const cv::Mat &maskSrc)
{
	assert(maskSrc.type()==CV_8UC1);
	mask = maskSrc.clone();

	rgbImageBuf[0] = cv::Mat(mask.rows, mask.cols, CV_8UC1);
	rgbImageBuf[1] = cv::Mat(mask.rows, mask.cols, CV_8UC1);
	rgbImageBuf[2] = cv::Mat(mask.rows, mask.cols, CV_8UC1);
}


/*
cv::Mat
ImagePreprocessor::preprocess(const cv::Mat &src)
{

}
*/


void
ImagePreprocessor::preprocess(cv::Mat &srcInplace)
const
{
	switch(pMode) {
	case AS_IS:
		break;
	case AGC:
		srcInplace = autoAdjustGammaRGB(srcInplace, mask);
		break;
	case ILLUMINATI:
		srcInplace = toIlluminatiInvariant(srcInplace, iAlpha);
		break;
	}
}


cv::Mat
ImagePreprocessor::histogram (cv::Mat &inputMono, cv::Mat mask)
{
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
//	uint8_t
	cv::calcHist (&inputMono, 1, 0, mask, hist, 1, &histSize, ranges, true, false);
	return hist;
}


float
ImagePreprocessor::detectSmear(cv::Mat &rgbImage, const float tolerance)
{
	// 1. Convert image to HSV and take V channel -> V
	cv::Mat imgHsv, V;
	cv::cvtColor(rgbImage, imgHsv, CV_BGR2HSV);
	cv::extractChannel(imgHsv, V, 2);
	// 2. Normalize V
	V.convertTo(V, CV_32F);
	V /= 255.0;
	// Sum all elements of V per column
	cv::Mat tv = cv::Mat::zeros(V.cols, 1, CV_32F);
	for (int i=0; i<V.cols; i++) {
		tv.at<float>(i) = cv::sum(V.col(i))[0];
	}
	tv /= (float)V.rows;
	// Count number of columns that are out of range
	const float th = 0.1;
	int nc = 0;
	for (int i=0; i<V.cols; i++) {
		if (tv.at<float>(i) >= 1.0-th)
			nc += 1;
	}
	// done
	if (nc < 2)
		return -1;
	else {
		float threshold = 0.15 * rgbImage.cols;
		if (nc > threshold)
			return 1.0;
		else
			return float(nc) / threshold;
	}
}


cv::Mat ImagePreprocessor::toIlluminatiInvariant (const cv::Mat &rgbImage, const float alpha)
{
	cv::Mat iImage (rgbImage.rows, rgbImage.cols, CV_8UC1);
//	cout << rgbImage.rows << ' ' << rgbImage.cols << endl;

	cv::MatConstIterator_<cv::Vec3b> it, end;
	for (it=rgbImage.begin<cv::Vec3b>(), end=rgbImage.end<cv::Vec3b>(); it!=end; ++it) {
//		cv::Vec3b &curPixel = *it;
		float
			fb = (*it)[0] / 255.0,
			fg = (*it)[1] / 255.0,
			fr = (*it)[2] / 255.0;
		float iPix = 0.5 + logf(fg) - alpha*logf(fb) - (1-alpha)*logf(fr);
		iImage.at<uchar>(it.pos()) = (uchar)(iPix*255);
	}

	return iImage;
}


cv::Mat ImagePreprocessor::setGamma (const cv::Mat &grayImage, const float gamma, bool LUT_only)
{
	cv::Mat grayOut;
	cv::Mat LUT = cv::Mat::zeros(1,256,CV_8UC1);
	for (int i=0; i<256; i++) {
		float v = (float)i / 255;
		v = powf(v, gamma);
		LUT.at<uchar>(i) = cv::saturate_cast<uchar>(v*255);
	}
	if (LUT_only)
		return LUT.clone();
	cv::LUT(grayImage, LUT, grayOut);

	return grayOut;
}


cv::Mat ImagePreprocessor::autoAdjustGammaMono(cv::Mat &grayImg, float *gamma, cv::Mat mask)
{
	cv::Mat roicdf = cdf (grayImg, mask);

	float midtone = 0;
	for (int i=0; i<256; i++) {
		if (roicdf.at<float>(i) >= 0.5) {
			midtone = (float)i / 255.0;
			break;
		}
	}

	// no changes if proper/over-exposed
	// XXX: Need to review this
	if (midtone >= 0.5)
		return grayImg;

	float g = logf (0.5) / logf(midtone);
	if (gamma != NULL) {
		*gamma = g;
		return cv::Mat();
	}
	return setGamma(grayImg, g);
}


/*
 * XXX: Can we add histogram equalization here?
 * The point is to enhance image contrast
 */
cv::Mat ImagePreprocessor::autoAdjustGammaRGB (const cv::Mat &rgbImg, const cv::Mat &mask)
{
	cv::Mat res;
	cv::Mat monoImg;

	cv::cvtColor (rgbImg, monoImg, CV_BGR2GRAY);
	float gamma;
	autoAdjustGammaMono (monoImg, &gamma, mask);
	cv::Mat LUT = setGamma (monoImg, gamma, true);
	cv::LUT(monoImg, LUT, monoImg);

	cv::Mat histAll = histogram(monoImg);
	int i=0;
	while (!histAll.at<uchar>(i))
		i++;
	float a = 127.0/(127.0-(float)i);
	float b = -a*i;
	for (i=0; i<=127; i++) {
		uchar &u = LUT.at<uchar>(i);
		u = a*u + b;
	}

	vector<cv::Mat> rgbBuf;
	cv::split (rgbImg, rgbBuf);
//	cv::LUT(rgbBuf[0], LUT, rgbBuf[0]);
//	cv::LUT(rgbBuf[1], LUT, rgbBuf[1]);
//	cv::LUT(rgbBuf[2], LUT, rgbBuf[2]);
	rgbBuf[0] = setGamma (rgbBuf[0], gamma);
	rgbBuf[1] = setGamma (rgbBuf[1], gamma);
	rgbBuf[2] = setGamma (rgbBuf[2], gamma);
//	cv::equalizeHist(rgbBuf[0], rgbBuf[0]);
//	cv::equalizeHist(rgbBuf[1], rgbBuf[1]);
//	cv::equalizeHist(rgbBuf[2], rgbBuf[2]);

	cv::Mat BGRres;
	cv::merge (rgbBuf, BGRres);
	return BGRres;
}


/**
 * Don't use this
 */
/*
cv::Mat ImagePreprocessor::autoAdjustGammaRGB(cv::Mat &rgbImg, cv::Mat mask)
{
	cv::Mat res;
	cv::Mat monoImg;

	cv::cvtColor (rgbImg, monoImg, CV_BGR2GRAY);
	float gamma;
	autoAdjustGammaMono (monoImg, &gamma, mask);

	vector<cv::Mat> BGRimg(3);
	cv::split(rgbImg, BGRimg);

	BGRimg[0] = setGamma (BGRimg[0], gamma);
	BGRimg[1] = setGamma (BGRimg[1], gamma);
	BGRimg[2] = setGamma (BGRimg[2], gamma);

	cv::Mat BGRres;
	cv::merge(BGRimg, BGRres);
	return BGRres;
}
*/


cv::Mat ImagePreprocessor::cdf (cv::Mat &grayImage, cv::Mat mask)
{
	cv::Mat rcdf = cv::Mat::zeros(1,256,CV_32F);
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
	cv::calcHist (&grayImage, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);
	// cumulative sum
	rcdf.at<float>(0) = hist.at<float>(0);
	for (int i=1; i<histSize; i++) {
		rcdf.at<float>(i) = rcdf.at<float>(i-1) + hist.at<float>(i);
	}

	rcdf = rcdf / cv::sum(hist)[0];
	return rcdf;
}
