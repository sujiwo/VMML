#include <iostream>
#include <array>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include "im_enhance.h"
#include "timer.h"
#include "matutils.h"


using namespace std;


namespace ice {


/*
 * Copied from https://github.com/WurmD/LowPass/blob/master/LowPassV.cpp
 * Also from : https://github.com/RoyiAvital/FastGuassianBlur.git
 */
vector<int> boxesForGauss(float sigma, int n)  // standard deviation, number of boxes
{
	auto wIdeal = sqrt((12 * sigma*sigma / n) + 1);  // Ideal averaging filter width
	int wl = floor(wIdeal);
	if (wl % 2 == 0)
		wl--;
	int wu = wl + 2;

	auto mIdeal = (12 * sigma*sigma - n * wl*wl - 4 * n*wl - 3 * n) / (-4 * wl - 4);
	int m = round(mIdeal);

	vector<int> sizes(n);
	for (auto i = 0; i < n; i++)
		sizes[i] = i < m ? wl : wu;
	return sizes;
}


void boxBlurH(cv::Mat &src, cv::Mat &dst, int radius)
{
	float iarr = 1.f / (2*radius + 1);

	// Point of parallelism
	for (auto i=0; i<src.rows; ++i) {

		int ti = 0, li = 0;
		auto ri = int(radius);
		auto fv = src.at<float>(i, 0), lv = src.at<float>(i, src.cols-1);
		auto val = (radius+1) * fv;

		for (auto j=0; j<radius; ++j) {
			val += src.at<float>(i, j);
		}

		for (auto j=0; j<=radius; ++j) {
			val += src.at<float>(i, ri++) - fv;
			dst.at<float>(i, ti++) = val * iarr;
		}

		for (auto j=radius+1; j<src.cols-radius; ++j) {
			val += src.at<float>(i, ri++) - src.at<float>(i, li++);
			dst.at<float>(i, ti++) = val*iarr;
		}

		for (auto j=src.cols - radius; j<src.cols; ++j) {
			val+= lv - src.at<float>(i, li++);
			dst.at<float>(i, ti++) = val*iarr;
		}
	}
}


void boxBlurT(cv::Mat &src, cv::Mat &dst, int radius)
{
	float iarr = 1.f / (2*radius +1);

	// Point of parallelism
	for (auto i=0; i<src.cols; ++i) {

		int ti=i, li=i;
		auto ri = int(radius);
		auto fv = src.at<float>(0, i), lv = src.at<float>(src.rows-1, i);
		auto val = (radius+1) * fv;

		for (auto j=0; j<radius; ++j) {
			val += src.at<float>(j, i);
		}

		for (auto j=0; j<radius; ++j) {
			val += src.at<float>(j, ri++) - fv;
//			dst.at<float>()
		}
	}
}


cv::Mat fastGaussianBlur (const cv::Mat &input, float r)
{
	// create box filter
}


void log10(const Matf &A, Matf &B)
{
	if (B.size()!=A.size())
		B.create(A.size());

	auto Bit = B.begin();
	for (auto Ait=A.begin(); Ait!=A.end(); ++Ait) {
		*Bit = log10f(*Ait);
		Bit++;
	}
}


/*
 * Retinex Family
 */
Matf
singleScaleRetinex(const Matf &inp, const float sigma, bool useFaster=false)
{
	Matf inpLog;
	log10(inp, inpLog);

	// GaussianBlur() is also a hotspot for large sigma
	Matf blurred;
	if (useFaster==false)
		cv::GaussianBlur(inp, blurred, cv::Size(0,0), sigma);

	else {
		auto boxes = boxesForGauss(sigma, 3);
		cv::boxFilter( inp, blurred, -1, cv::Size(boxes[0]/2, boxes[0]/2) );
		cv::boxFilter( blurred, blurred, -1, cv::Size(boxes[1]/2, boxes[1]/2) );
		cv::boxFilter( blurred, blurred, -1, cv::Size(boxes[2]/2, boxes[2]/2) );
	}

	log10(blurred, blurred);

	auto R=inpLog - blurred;
	return R;
}


/*
 * GPU version of Single-scale retinex
 */
cv::UMat
singleScaleRetinex(const cv::UMat &inp, const float sigma)
{
	assert(inp.type()==CV_32FC1);

	// XXX: log_e or log_10 ?
	cv::UMat inpLog;
	cv::log(inp, inpLog);
	cv::multiply(1.0/logf(10), inpLog, inpLog);

	cv::UMat gaussBlur;
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);

	cv::log(gaussBlur, gaussBlur);
	cv::multiply(1.0/logf(10), gaussBlur, gaussBlur);

	cv::UMat R;
	cv::subtract(inpLog, gaussBlur, R);
	return R;
}


cv::Mat
multiScaleRetinex(const cv::Mat &inp, const float sigma1,
		const float sigma2,
		const float sigma3)
{
	cv::Mat msrex = cv::Mat::zeros(inp.size(), CV_32FC1);
	double mmin, mmax;

/*
	array<float,3> _sigmaList = {sigma1, sigma2, sigma3};
	for (auto &s: _sigmaList) {
		cv::Mat ssRetx = singleScaleRetinex(inp, s);
		msrex = msrex + ssRetx;
	}
*/

	cv::Mat ssRetx = singleScaleRetinex(inp, sigma1);
	msrex = msrex + ssRetx;
	ssRetx = singleScaleRetinex(inp, sigma2, true);
	msrex = msrex + ssRetx;
	ssRetx = singleScaleRetinex(inp, sigma3, true);
	msrex = msrex + ssRetx;

	msrex /= 3;
//	npy::saveMat(msrex, "/tmp/retinexac.npy"); exit(-1);
	return msrex;
}


cv::Mat
multiScaleRetinexGpu(const cv::Mat &inp,
		const float sigma1,
		const float sigma2,
		const float sigma3)
{
	cv::UMat
		msrex = cv::UMat::zeros(inp.size(), CV_32FC1),
		inputg;
	inp.copyTo(inputg);

	array<float,3> _sigmaList = {sigma1, sigma2, sigma3};
	for (auto &s: _sigmaList) {
		auto t1 = getCurrentTime();
		cv::UMat ssRetx = singleScaleRetinex(inputg, s);
		auto t2 = getCurrentTime();
		cout << "Retinex " << s << ": " << to_seconds(t2-t1) << endl;
		cv::add(msrex, ssRetx, msrex);
	}
	cv::multiply(1.0/3.0, msrex, msrex);

	cv::Mat outp;
	msrex.copyTo(outp);
	return outp;
}


Matf
simpleColorBalance2(const Matf &inp, const float lowClip, const float highClip)
{
	uint current = 0;
	const uint total = inp.total();
	double low_val, high_val;

	std::vector<float> uniquez(inp.begin(), inp.end());
	std::sort(uniquez.begin(), uniquez.end());
	int clow = floor(float(lowClip) * float(total));
	int chigh = floor(float(highClip) * float(total));
	low_val = uniquez[clow];
	high_val = uniquez[chigh];

	Matf minImg, maxImg;
	cv::min(inp, high_val, minImg);
	cv::max(minImg, low_val, maxImg);

	return maxImg;
}



Matf
simpleColorBalance(const Matf &inp, const float lowClip, const float highClip)
{
	uint current = 0;
	const uint total = inp.total();
	double low_val, high_val;

	auto mUnique = unique(inp);
	auto uniqVals = mUnique.getKeys();
	auto uniqCounts = mUnique.getValues();
	float f1 = uniqVals.front(), f2 = uniqVals.back();

	for (int i=0; i<uniqVals.size(); ++i) {
		auto u = uniqVals.at(i);
		auto c = uniqCounts.at(i);
		if (float(current) / total < lowClip)
			low_val = u;
		if (float(current) / total < highClip)
			high_val = u;
		current += c;
	}

	Matf minImg, maxImg;
	cv::min(inp, high_val, minImg);
	cv::max(minImg, low_val, maxImg);

	return maxImg;
}


cv::Mat multiScaleRetinexCP(const cv::Mat &rgbImage,
	const float sigma1,
	const float sigma2,
	const float sigma3,
	const float lowClip, const float highClip)
{
//	cv::ocl::setUseOpenCL(true);
	Matf3 imgf;
	rgbImage.convertTo(imgf, CV_32FC3, 1.0, 1.0);

	Matf intensity (imgf.size(), CV_32FC1);
	auto bit = intensity.begin();
	for (auto ait=imgf.begin(); ait!=imgf.end(); ++ait) {
		auto color = *ait;
		*bit = (color[0] + color[1] + color[2]) / 3;
		++bit;
	}

	auto t1 = getCurrentTime();
	Matf firstRetinex = multiScaleRetinex(intensity, sigma1, sigma2, sigma3);
	auto t2 = getCurrentTime();
//	cout << "MSR: " << to_seconds(t2-t1) << endl;

	Matf intensity1 = simpleColorBalance2(firstRetinex, lowClip, highClip);

	double intensMin, intensMax;
	cv::minMaxIdx(intensity1, &intensMin, &intensMax);
	intensity1 = ((intensity1 - intensMin) / (intensMax - intensMin)) * 255.0 + 1.0;

	cv::Mat imgMsrcp (imgf.size(), imgf.type());
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto _B = imgf(r, c);
			auto B = max({_B[0], _B[1], _B[2]});
			auto A = min(float(256.0)/B, intensity1.at<float>(r,c) / intensity.at<float>(r,c));
			cv::Vec3f color;
			color[0] = A * imgf.at<cv::Vec3f>(r,c)[0];
			color[1] = A * imgf.at<cv::Vec3f>(r,c)[1];
			color[2] = A * imgf.at<cv::Vec3f>(r,c)[2];
			imgMsrcp.at<cv::Vec3f>(r,c) = color;
		}

	cv::Mat imgMsrcpInt8;
	imgMsrcp = imgMsrcp-1;
	imgMsrcp.convertTo(imgMsrcpInt8, CV_8UC3);

	return imgMsrcpInt8;
}

}		// namespace ice

