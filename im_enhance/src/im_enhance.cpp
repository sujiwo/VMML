#include <vector>
#include <array>
#include <iostream>
#include <exception>
#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>
#include "im_enhance.h"
#include "matutils.h"


using namespace std;


namespace ice {

cv::Mat
histogram (cv::Mat &inputMono, cv::InputArray mask=cv::noArray())
{
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
	cv::calcHist (&inputMono, 1, 0, mask, hist, 1, &histSize, ranges, true, false);
	return hist;
}


cv::Mat setGamma (const cv::Mat &grayImage, const float gamma, bool LUT_only=false)
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


Matf cdf (cv::Mat &grayImage, cv::Mat mask, bool normalized)
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

	if (normalized)
		rcdf = rcdf / cv::sum(hist)[0];
	return rcdf;
}


cv::Mat autoAdjustGammaMono(cv::Mat &grayImg, float *gamma, cv::Mat mask)
{
	cv::Mat roicdf = cdf (grayImg, mask);

	float midtone = 0;
	for (int i=0; i<256; i++) {
		if (roicdf.at<float>(i) >= 0.5) {
			midtone = (float)i / 255.0;
			break;
		}
	}

	float g = logf (0.5) / logf(midtone);

	// no changes if proper/over-exposed
	if (midtone >= 0.5)
		g = 1.0;

	if (gamma != NULL) {
		*gamma = g;
		return cv::Mat();
	}
	return setGamma(grayImg, g);
}


cv::Mat autoAdjustGammaRGB (const cv::Mat &rgbImg, cv::InputArray mask)
{
	cv::Mat res;
	cv::Mat monoImg;

	cv::cvtColor (rgbImg, monoImg, CV_BGR2GRAY);

	float gamma;
	autoAdjustGammaMono (monoImg, &gamma, mask.getMat());
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
	rgbBuf[0] = setGamma (rgbBuf[0], gamma);
	rgbBuf[1] = setGamma (rgbBuf[1], gamma);
	rgbBuf[2] = setGamma (rgbBuf[2], gamma);

	cv::Mat BGRres;
	cv::merge (rgbBuf, BGRres);
	return BGRres;
}


cv::Mat toIlluminatiInvariant (const cv::Mat &rgbImage, const float alpha)
{
	Matc3 rgbc (rgbImage);
	Matc3 iImage (rgbImage.rows, rgbImage.cols);

	auto oImgIt = iImage.begin();
	for (auto it=rgbc.begin(); it!=rgbc.end(); ++it) {
		float fb, fg, fr;
			fb = (*it)[0] / 255.0;
			fg = (*it)[1] / 255.0;
			fr = (*it)[2] / 255.0;
		float iPix = 0.5 + logf(fg) - alpha*logf(fb) - (1-alpha)*logf(fr);
		(*oImgIt) = (uchar)(iPix*255);
		oImgIt++;
	}

	return iImage;
}










}		// namespace ice

