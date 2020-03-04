/*
 * Retinex.cpp
 *
 *  Created on: Feb 10, 2020
 *      Author: sujiwo
 *
 */

#include <set>
#include <map>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/hdf.hpp>
#include <opencv2/core/ocl.hpp>
#include "vmml/Retinex.h"
#include "vmml/utilities.h"


using namespace std;


namespace Vmml {


cv::Mat
Retinex::singleScaleRetinex(const cv::Mat &inp, const float sigma)
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
Retinex::multiScaleRetinex(const cv::Mat &inp, const std::array<float,3> _sigmaList)
{
	cv::Mat msrex = cv::Mat::zeros(inp.size(), CV_32FC1);
	double mmin, mmax;

	for (auto &s: _sigmaList) {
		cv::Mat ssRetx = singleScaleRetinex(inp, s);
		msrex = msrex + ssRetx;
	}

	msrex /= 3;

	return msrex;
}


cv::Mat
Retinex::simpleColorBalance(const cv::Mat &inp, const float lowClip, const float highClip)
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

/*
	std::set<float> unique;
	std::map<float, uint> counter;
	for (auto it=inp.begin<float>(); it!=inp.end<float>(); ++it) {
		auto &u = *it;
		auto rt = unique.emplace(u);
		if (rt.second==true)
			counter[u] = 1;
		else
			counter[u] += 1;
	}
	assert(unique.size()==counter.size());

	for (auto it=unique.begin(); it!=unique.end(); ++it) {
		auto &u = *it;
		auto c = counter[u];
		if (float(current)/float(total) < lowClip)
			low_val = u;
		if (float(current)/float(total) < highClip)
			high_val = u;
		current += c;
	}
*/

	cv::Mat minImg, maxImg;
	cv::min(inp, high_val, minImg);
	cv::max(minImg, low_val, maxImg);

	return maxImg;
}


cv::Mat
Retinex::run(const cv::Mat &input)
{
	cv::Mat imgf;
	input.convertTo(imgf, CV_32FC3, 1.0, 1.0);

	auto t1=Vmml::getCurrentTime();
	cv::Mat intensity (imgf.size(), CV_32FC1);
	for (uint r=0; r<imgf.rows; ++r)
		for (uint c=0; c<imgf.cols; ++c) {
			auto color = imgf.at<cv::Vec3f>(r,c);
			intensity.at<float>(r,c) = (color[0]+color[1]+color[2])/3;
		}
	auto t2=Vmml::getCurrentTime();
	cerr << "Time 1: " << Vmml::toSeconds(t2-t1) << endl;

	/*
	 * Timer 2 & 3 are slowest
	 */

	t1=Vmml::getCurrentTime();
	cv::Mat firstRetinex = multiScaleRetinex(intensity, sigma);
	t2=Vmml::getCurrentTime();
	cerr << "Time 2: " << Vmml::toSeconds(t2-t1) << endl;

	t1=Vmml::getCurrentTime();
	cv::Mat intensity1 = simpleColorBalance(firstRetinex, low_clip, high_clip);
	t2=Vmml::getCurrentTime();
	cerr << "Time 3: " << Vmml::toSeconds(t2-t1) << endl;

	// XXX: intensMin & max produces zero values. There may be errors in multiScaleRetinex(),
	// need to dump matrix values to numpy and analyze them using python by imshow(),
	// because their values are in log domain
	double intensMin, intensMax;
	cv::minMaxIdx(intensity1, &intensMin, &intensMax);
	intensity1 = (intensity1 - intensMin) / (intensMax - intensMin) * 255.0 + 1.0;

	t1=Vmml::getCurrentTime();
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
	t2=Vmml::getCurrentTime();
	cerr << "Time 4: " << Vmml::toSeconds(t2-t1) << endl;

	cv::Mat imgMsrcpInt8;
	imgMsrcp = imgMsrcp-1;
	imgMsrcp.convertTo(imgMsrcpInt8, CV_8UC3);

	return imgMsrcpInt8;
}








}	// namespace Vmml
