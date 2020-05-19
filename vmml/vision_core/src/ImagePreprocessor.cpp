/*
 * ImagePreprocessor.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: sujiwo
 */

#include <vector>
#include <opencv2/bioinspired.hpp>
#include <opencv2/xphoto.hpp>
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


template<typename T>
T clamp(const T &v, const T &min, const T &max=numeric_limits<T>::max())
{
	return (v<min ? min : (v>max ? max : v));
}


template<typename T, typename U>
T clamp(const U &v, const U &min=numeric_limits<T>::min(), const U &max=numeric_limits<T>::max())
{
	U vcrop = (v<min ? min : (v>max ? max : v));
	return (T)v;
}


cv::Mat ImagePreprocessor::toIlluminatiInvariant (const cv::Mat &imageBayer, const float alpha)
{
	cv::Mat iImage (imageBayer.rows/2, imageBayer.cols/2, CV_8UC1);
	cv::Mat greyf (imageBayer.rows/2, imageBayer.cols/2, CV_32F);

	for (int i=0; i<greyf.rows; i++) {
		for (int j=0; j<greyf.cols; j++) {
			auto pr=i*2;
			auto pc=j*2;
			float g1=float(clamp(imageBayer.at<uchar>(pr,pc), uchar(1), uchar(255))) / 256.0;
			float r=float(imageBayer.at<uchar>(pr,pc+1) +1) / 256.0;
			float b=float(imageBayer.at<uchar>(pr+1,pc) +1) / 256.0;
			float g2=float(imageBayer.at<uchar>(pr+1,pc+1) +1) / 256.0;
			float g=(g1+g2)/2.0;
			float iv = 0.5 + log10f(g) - alpha*log10f(b) - (1-alpha)*log10f(r);
			greyf.at<float>(i,j) = clamp(iv, 0.0f, 1.0f);
		}
	}

	// Normalize
	double maxVal;
	cv::minMaxLoc(greyf, NULL, &maxVal);
	greyf = greyf *256.0;
	greyf.convertTo(iImage, iImage.type());
	return iImage;
}

cv::Mat ImagePreprocessor::toIlluminatiInvariantRGB (const cv::Mat &rgbImage, const float alpha)
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


cv::Mat
ImagePreprocessor::retinaHdr(const cv::Mat &rgbImage, cv::Vec3f weights, cv::Vec3f sigmas, int gain, int offset, double restoration_factor, double color_gain)
{
	cv::Mat A, B, C, fA, fB, fC, fsA, fsB, fsC, fsD, fsE, fsF;

	if (weights[0]==-1 and weights[1]==-1 and weights[2]==-1)
		weights[0]=weights[1]=weights[2]=1.0/float(rgbImage.channels());

	rgbImage.convertTo(fB, CV_32F);
	cv::log(fB, fA);

	float weight = weights[0]+weights[1]+weights[2];
	if (weight != 1.0)
		fA.convertTo(fA, -1, weight);

	// Filter at each scale
	for (int i=0; i<3; i++) {
		A = rgbImage.clone();
		cv::GaussianBlur(A, A, cv::Size(0,0), sigmas[i]);

		A.convertTo(fB, fB.type());
		cv::log(fB, fC);

		// compute weighted difference
		fC = fC * weights[i];
		fA = fA - fC;
	}

	// Color restoration
	{
		vector<cv::Mat> rgbs;

		// Divide image into channels, convert and store sum
		cv::split(rgbImage, rgbs);
		rgbs[0].convertTo(fsA, CV_32F);
		rgbs[1].convertTo(fsB, CV_32F);
		rgbs[2].convertTo(fsC, CV_32F);

		// Normalize weights
		fsD = fsA + fsB + fsC;
		cv::divide(fsA, fsD, fsA, restoration_factor);
		cv::divide(fsB, fsD, fsB, restoration_factor);
		cv::divide(fsC, fsD, fsC, restoration_factor);

		fsA.convertTo(fsA, -1, 1, 1);
		fsB.convertTo(fsB, -1, 1, 1);
		fsC.convertTo(fsC, -1, 1, 1);

		// Log weights
		cv::log(fsA, fsA);
		cv::log(fsB, fsB);
		cv::log(fsC, fsC);

		// Divide retinex image, weight accordingly and recombine
		vector<cv::Mat> defs;
		cv::split(fA, defs);

		cv::multiply(defs[0], fsA, defs[0], color_gain);
		cv::multiply(defs[1], fsB, defs[1], color_gain);
		cv::multiply(defs[2], fsC, defs[2], color_gain);

		cv::merge(defs, fA);
	}

	// Restore
	cv::Mat RetImg;
	fA.convertTo(RetImg, CV_8UC3, gain, offset);

	return RetImg;
}


cv::Mat
ImagePreprocessor::GrayWorld(const cv::Mat &bgrImage)
{
	double B=0, G=0, R=0;
	cv::Mat dsRes(bgrImage.size(), CV_8UC3);

	for (int i=0; i<bgrImage.rows; i++) {
		for (int j=0; j<bgrImage.cols; j++) {
			auto color = bgrImage.at<cv::Vec3b>(i, j);
			B += float(color[0]);
			G += float(color[1]);
			R += float(color[2]);
		}
	}

	B /= bgrImage.cols*bgrImage.rows;
	G /= bgrImage.cols*bgrImage.rows;
	R /= bgrImage.cols*bgrImage.rows;
	double grayVal = (B+G+R)/3;
	double
		kr = grayVal / R,
		kg = grayVal / G,
		kb = grayVal / B;

	for (int i=0; i<bgrImage.rows; ++i) {
		for (int j=0; j<bgrImage.cols; ++j) {
			auto colorSrc = bgrImage.at<cv::Vec3b>(i, j);
			cv::Vec3b colorDst;
			colorDst[0] = clamp<uchar>(colorSrc[0] * kb);
			colorDst[1] = clamp<uchar>(colorSrc[1] * kg);
			colorDst[2] = clamp<uchar>(colorSrc[2] * kr);
			dsRes.at<cv::Vec3b>(i, j) = colorDst;
		}
	}

	return dsRes;
}
