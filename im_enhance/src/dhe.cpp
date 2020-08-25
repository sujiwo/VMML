/*
 * dhe.cpp
 *
 *  Created on: Aug 25, 2020
 *      Author: sujiwo
 */

#include <opencv2/imgproc.hpp>
#include "im_enhance.h"
#include "matutils.h"
#include "timer.h"


namespace ice {

cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha)
{

}


void build_is_histogram(const cv::Mat &bgrImage, cv::OutputArray hist_i, cv::OutputArray hist_s)
{
	const int
		height = bgrImage.rows,
		width = bgrImage.cols,
		channel = bgrImage.channels();

	// replace numpy.pad to edge with Opencv copyMakeBorder(), BORDER_REPLICATE

	Matf3 rgbf, hsv;
	bgrImage.convertTo(rgbf, CV_32FC3);
	cv::cvtColor(rgbf, hsv, cv::COLOR_BGR2HSV);

	Matf HSI[3];
	cv::split(hsv, HSI);
	Matf I = HSI[2], S = HSI[1];

	// fh and fv rotated
	Matf fhr(3,3), fvr(3,3);
	fhr << 1,0,-1,
			2,0,-2,
			1,0,-1;
	fvr << 1,2,1,
			0,0,0,
			-1,-2,-1;

	Matf dIh, dIv, dI;
	cv::filter2D(I, dIh, CV_32F, fhr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	cv::filter2D(I, dIv, CV_32F, fvr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	dIh.setTo(1e-3, dIh==0);
	dIv.setTo(1e-3, dIv==0);
	cv::pow(dIh, 2, dIh);
	cv::pow(dIv, 2, dIv);
	dI = dIh+dIv;
	cv::sqrt(dI, dI);
	Matui dIint(dI.size());
	std::transform(dI.begin(), dI.end(), dIint.begin(),
		[](const float &f){ return uint32_t(f); }
	);

	Matf dSh, dSv, dS;
	cv::filter2D(S, dSh, CV_32F, fhr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	cv::filter2D(S, dSv, CV_32F, fvr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	dSh.setTo(1e-3, dSh==0);
	dSv.setTo(1e-3, dSv==0);
	cv::pow(dSh, 2, dSh);
	cv::pow(dSv, 2, dSv);
	dS = dSh+dSv;
	cv::sqrt(dS, dS);
	Matui dSint(dS.size());
	std::transform(dS.begin(), dS.end(), dSint.begin(),
		[](const float &f){ return uint32_t(f); }
	);

	Matf Imean, Smean;
	cv::boxFilter(I, Imean, CV_32F, cv::Size(5,5), cv::Point(-1,-1), true);
	cv::boxFilter(S, Smean, CV_32F, cv::Size(5,5), cv::Point(-1,-1), true);

	Matf Rho = Matf::zeros(bgrImage.size());
	for (auto r=0; r<Rho.rows; ++r) {
		for (auto c=0; c<Rho.cols; ++c) {

		}
	}
}


}		// namespace ice
