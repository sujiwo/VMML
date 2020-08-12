#include <iostream>
#include <array>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include "im_enhance.h"


using namespace std;


namespace ice {

/*
 * Retinex Family
 */
cv::Mat
singleScaleRetinex(const cv::Mat &inp, const float sigma)
{
	assert(inp.type()==CV_32FC1);

	// XXX: log_e or log_10 ?
	cv::Mat inpLog;
	cv::log(inp, inpLog);
	inpLog /= log(10);

	// GaussianBlur() is also a hotspot for large sigma
	cv::Mat gaussBlur;
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);

	cv::log(gaussBlur, gaussBlur);
	gaussBlur /= log(10);

	auto R=inpLog - gaussBlur;
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

	array<float,3> _sigmaList = {sigma1, sigma2, sigma3};
	for (auto &s: _sigmaList) {
		cv::Mat ssRetx = singleScaleRetinex(inp, s);
		msrex = msrex + ssRetx;
	}

	msrex /= 3;
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


cv::Mat
simpleColorBalance(const cv::Mat &inp, const float lowClip, const float highClip)
{
	assert(inp.type()==CV_32F);

	const uint total = inp.total();
	uint current = 0;
	double low_val, high_val;

	// This part is hotspot
	auto t1 = getCurrentTime();
	std::vector<float> uniquez(inp.begin<float>(), inp.end<float>());
	std::sort(uniquez.begin(), uniquez.end());
	auto t2 = getCurrentTime();
	cout << "Unique: " << to_seconds(t2-t1) << endl;
	int clow = floor(float(lowClip) * float(total));
	int chigh = floor(float(highClip) * float(total));
	low_val = uniquez[clow];
	high_val = uniquez[chigh];

	cv::Mat minImg, maxImg;
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
	cv::ocl::setUseOpenCL(true);
	cv::Mat imgf;
	rgbImage.convertTo(imgf, CV_32FC3, 1.0, 1.0);

	cv::Mat intensity (imgf.size(), CV_32FC1);
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto color = imgf.at<cv::Vec3f>(r,c);
			intensity.at<float>(r,c) = (color[0]+color[1]+color[2])/3;
		}

	auto t1 = getCurrentTime();
	cv::Mat firstRetinex = multiScaleRetinexGpu(intensity, sigma1, sigma2, sigma3);
	auto t2 = getCurrentTime();
	cout << "MSR: " << to_seconds(t2-t1) << endl;

	cv::Mat intensity1 = simpleColorBalance(firstRetinex, lowClip, highClip);

	double intensMin, intensMax;
	cv::minMaxIdx(intensity1, &intensMin, &intensMax);
	intensity1 = (intensity1 - intensMin) / (intensMax - intensMin) * 255.0 + 1.0;

	cv::Mat imgMsrcp (imgf.size(), imgf.type());
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto _B = imgf.at<cv::Vec3f>(r, c);
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

