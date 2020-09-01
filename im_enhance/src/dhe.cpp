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
#include "npy.hpp"


using namespace std;


namespace ice {


void build_is_histogram(const cv::Mat &bgrImage, Matf &hist_i, Matf &hist_s);
cv::Mat matlab_covar(const cv::Mat &A, const cv::Mat &B);
cv::Mat corrcoeff (const cv::Mat &M1, const cv::Mat &M2);


/*
 * Notes: Comparison of OpenCV vs Matplotlib RGB->HSV conversion
 */




cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha)
{
	// Work in HSV
	Matf3 rgbf, hsv;
	rgbImage.convertTo(rgbf, CV_32FC3);
	cv::cvtColor(rgbf, hsv, cv::COLOR_BGR2HSV);
	Matf HSI[3];
	cv::split(hsv, HSI);
	Matf I = HSI[2], S = HSI[1], H = HSI[0];

	Matf hist_i, hist_s;
	build_is_histogram(rgbImage, hist_i, hist_s);



	// XXX: Unfinished
	exit(-1);
}


/*
 * Compute Covariance matrix ala Matlab & NumPy.
 * XXX: Answer is different
 */
cv::Mat matlab_covar(const cv::Mat &A, const cv::Mat &B)
{
	assert(A.cols*A.rows == B.cols*B.rows);
//	const vector<cv::Mat> Inp = {A.reshape(0, A.cols*A.rows).t(), B.reshape(0, B.cols*B.rows).t()};
	cv::Mat Az, covar, mean;
	cv::vconcat(A.reshape(0, A.cols*A.rows).t(), B.reshape(0, B.cols*B.rows).t(), Az);
	cv::calcCovarMatrix(Az, covar, mean, cv::COVAR_COLS|cv::COVAR_NORMAL);
	covar /= (Az.cols-1);
	return covar;
}


cv::Mat corrcoeff (const cv::Mat &M1, const cv::Mat &M2)
{
	assert(M1.channels()==1 and M2.channels()==1);

	auto cov = matlab_covar(M1, M2);
	auto d = cov.diag();
	cv::sqrt(d, d);

	for (int i=0; i<cov.rows; ++i)
		cov.row(i) /= d.t();
	for (int i=0; i<cov.cols; ++i)
		cov.col(i) /= d;
	cov.setTo(-1, cov< -1);
	cov.setTo(1, cov>1);

	return cov;
}


void build_is_histogram(const cv::Mat &bgrImage, Matf &hist_i, Matf &hist_s)
{
	const int
		height = bgrImage.rows,
		width = bgrImage.cols,
		channel = bgrImage.channels();

	Matf3 rgbf, hsv;
	bgrImage.convertTo(rgbf, CV_32FC3);
	cv::cvtColor(rgbf, hsv, cv::COLOR_BGR2HSV);

	Matf HSI[3];
	cv::split(hsv, HSI);
	Matf I = HSI[2], S = HSI[1], H = HSI[0];
	H *= 255.0;
	S *= 255.0;

	// fh and fv are already rotated and flipped
	Matf fhr(3,3), fvr(3,3);
	fhr << -1,0,1,
			-2,0,2,
			-1,0,1;
	fvr << -1,-2,-1,
			0,0,0,
			1,2,1;

	Matf dIh, dIv, dI;
	cv::filter2D(I, dIh, CV_32F, fhr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	cv::filter2D(I, dIv, CV_32F, fvr, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
	dIh.setTo(1e-3, dIh==0);
	dIv.setTo(1e-3, dIv==0);
	cv::pow(dIh, 2, dIh);
	cv::pow(dIv, 2, dIv);
	dI = dIh+dIv;
	cv::sqrt(dI, dI);
	Mati dIint(dI.size());
	dI.convertTo(dIint, CV_32SC1);

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

	auto t1=getCurrentTime();
	Matf Rho = Matf::zeros(bgrImage.size());
	for (auto r=0; r<Rho.rows; ++r) {
		for (auto c=0; c<Rho.cols; ++c) {
			auto tmpI = subMat(I, cv::Point(c,r), 5, 5);
			auto tmpS = subMat(S, cv::Point(c,r), 5, 5);
			tmpI = flatten(tmpI, 1);
			tmpS = flatten(tmpS, 1);
			auto corrv = corrcoeff(tmpI, tmpS);
			auto f = corrv.at<double>(0,1);
			f = fabs(f);
			if (isnan(f)) f=0;
			Rho(r,c) = f;
		}
	}
	auto t2=getCurrentTime();
	cout << "Correlations: " << to_seconds(t2-t1) << endl;

	cv::Mat rd = Rho.mul(dS);
	rd.convertTo(rd, CV_32S);

	hist_i = Matf::zeros(256,1);
	hist_s = Matf::zeros(256,1);

	Mati Intensity;
	I.convertTo(Intensity, CV_32SC1);
	for (auto n=0; n<255; ++n) {
		cv::Mat temp;
		dIint.copyTo(temp, Intensity==n);
		hist_i(n+1,0) = float(cv::sum(temp).val[0]);
		temp.setTo(0);
		rd.copyTo(temp, Intensity==n);
		hist_s(n+1,0) = float(cv::sum(temp).val[0]);
	}

	// Done?
}


}		// namespace ice
