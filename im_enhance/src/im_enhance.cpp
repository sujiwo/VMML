#include <vector>
#include <array>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "im_enhance.h"


using namespace std;


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


cv::Mat cdf (cv::Mat &grayImage, cv::Mat mask)
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
//	auto t1=Vmml::getCurrentTime();
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);
//	auto t2=Vmml::getCurrentTime();
//	cerr << "Time 1s: " << Vmml::toSeconds(t2-t1) << endl;

	cv::log(gaussBlur, gaussBlur);
	gaussBlur /= log(10);

	auto R=inpLog - gaussBlur;
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
simpleColorBalance(const cv::Mat &inp, const float lowClip, const float highClip)
{
	assert(inp.type()==CV_32F);

	const uint total = inp.total();
	uint current = 0;
	double low_val, high_val;

	// This part is hotspot
	std::vector<float> uniquez(inp.begin<float>(), inp.end<float>());
	std::sort(uniquez.begin(), uniquez.end());
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
	cv::Mat imgf;
	rgbImage.convertTo(imgf, CV_32FC3, 1.0, 1.0);

	cv::Mat intensity (imgf.size(), CV_32FC1);
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto color = imgf.at<cv::Vec3f>(r,c);
			intensity.at<float>(r,c) = (color[0]+color[1]+color[2])/3;
		}

	cv::Mat firstRetinex = multiScaleRetinex(intensity, sigma1, sigma2, sigma3);

	cv::Mat intensity1 = simpleColorBalance(firstRetinex, lowClip, highClip);

	// XXX: intensMin & max produces zero values. There may be errors in multiScaleRetinex(),
	// need to dump matrix values to numpy and analyze them using python by imshow(),
	// because their values are in log domain
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


void build_is_histogram(const cv::Mat &image, cv::OutputArray hist_i, cv::OutputArray hist_s)
{

}


cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha)
{

}




/*
 * Ying Et Al
 */
cv::Mat calculateWeightInput(const cv::Mat &rgbImage)
{
	cv::Mat rgbFloat, L, T(rgbImage.size(), CV_32FC1);

	cv::normalize(rgbImage, rgbFloat, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
	for (uint r=0; r<rgbFloat.rows; ++r) {
		for (uint c=0; c<rgbFloat.cols; ++c) {
			auto vc = rgbFloat.at<cv::Vec3f>(r,c);
			T.at<float>(r,c) = max(vc[0], max(vc[1], vc[2]));
		}
	}

	cv::resize(T, T, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
	cv::normalize(T, T, 0.0, 1.0, cv::NORM_MINMAX);

	// Calculate gradient (horizontal & vertical)

}

