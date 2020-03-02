/*
 * Retinex.cpp
 *
 *  Created on: Feb 10, 2020
 *      Author: sujiwo
 *
 *  XXX: This class is not functional
 */

#include <set>
#include <map>
#include <opencv2/imgproc.hpp>
#include <opencv2/hdf.hpp>
#include "vmml/Retinex.h"


using namespace std;


namespace Vmml {


cv::Mat
Retinex::singleScaleRetinex(const cv::Mat &inp, const double sigma)
{
	assert(inp.type()==CV_64FC1);

	// XXX: log_e or log_10 ?
	cv::Mat inpLog;
	cv::log(inp, inpLog);
	inpLog /= log(10);

	cv::Mat gaussBlur;
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);
	cv::log(gaussBlur, gaussBlur);
	gaussBlur /= log(10);

	auto R=inpLog - gaussBlur;
	return R;
}


cv::Mat
Retinex::multiScaleRetinex(const cv::Mat &inp, const std::array<double,3> _sigmaList)
{
	cv::Mat msrex = cv::Mat::zeros(inp.size(), CV_64FC1);
	double mmin, mmax;

	for (auto &s: _sigmaList) {
		cv::Mat ssRetx = singleScaleRetinex(inp, s);
		msrex = msrex + ssRetx;
	}

	msrex /= 3;
	return msrex;
}


cv::Mat
Retinex::simpleColorBalance(const cv::Mat &inp, const double lowClip, const double highClip)
{
	assert(inp.type()==CV_64F);

	const uint total = inp.total();
	uint current = 0;
	double low_val, high_val;

	std::set<double> unique;
	std::map<double, uint> counter;
	for (auto it=inp.begin<double>(); it!=inp.end<double>(); ++it) {
		auto &u = *it;
		auto rt = unique.emplace(u);
		if (rt.second==true)
			counter[u] = 1;
		else
			counter[u] += 1;
	}
	assert(unique.size()==counter.size());

	uint i=0;
	for (auto it=unique.begin(); it!=unique.end(); ++it) {
		auto &u = *it;
		auto c = counter[u];
		if (double(current)/double(total) < lowClip)
			low_val = u;
		if (double(current)/double(total) < highClip)
			high_val = u;
		current += c;
		++i;
	}

	cv::Mat minImg, maxImg;
	cv::min(inp, high_val, minImg);
	cv::max(minImg, low_val, maxImg);

	return maxImg;
}


cv::Mat
Retinex::run(const cv::Mat &input)
{
	cv::Mat imgf;
	input.convertTo(imgf, CV_64FC3, 1.0, 1.0);

	cv::Mat intensity (imgf.size(), CV_64F);
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto color = imgf.at<cv::Vec3d>(r,c);
			intensity.at<double>(r,c) = (color[0]+color[1]+color[2])/3;
		}

	cv::Mat firstRetinex = multiScaleRetinex(intensity, sigma);

	// C++ & Python version do not match
	cv::Mat intensity1 = simpleColorBalance(firstRetinex, low_clip, high_clip);

	// XXX: intensMin & max produces zero values. There may be errors in multiScaleRetinex(),
	// need to dump matrix values to numpy and analyze them using python by imshow(),
	// because their values are in log domain
	double intensMin, intensMax;
	cv::minMaxIdx(intensity1, &intensMin, &intensMax);
	intensity1 = (intensity1 - intensMin) / (intensMax - intensMin) * 255.0 + 1.0;

	cv::Mat imgMsrcp (imgf.size(), imgf.type());
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto _B = imgf.at<cv::Vec3d>(r, c);
			auto B = max({_B[0], _B[1], _B[2]});
			auto A = min(256.0/B, intensity1.at<double>(r,c) / intensity.at<double>(r,c));
			cv::Vec3d color;
			color[0] = A * imgf.at<cv::Vec3d>(r,c)[0];
			color[1] = A * imgf.at<cv::Vec3d>(r,c)[1];
			color[2] = A * imgf.at<cv::Vec3d>(r,c)[2];
			imgMsrcp.at<cv::Vec3d>(r,c) = color;
		}

	cv::Mat imgMsrcpInt8;
	imgMsrcp = imgMsrcp-1;
	imgMsrcp.convertTo(imgMsrcpInt8, CV_8UC3);

	return imgMsrcpInt8;
}








}	// namespace Vmml
