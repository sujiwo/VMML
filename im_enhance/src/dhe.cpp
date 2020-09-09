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


using namespace std;


namespace ice {


void build_is_histogram(const vector<Matf> &HSV, Matf &hist_i, Matf &hist_s);
cv::Mat matlab_covar(const cv::Mat &A, const cv::Mat &B);
cv::Mat corrcoeff (const cv::Mat &M1, const cv::Mat &M2);

/*
 * Output of BGR -> HSV:
 * Channel 0(H) -> [0...1]
 * Channel 1(S) -> [0...1]
 * Channel 2(V) -> [0...255]
 * All channels are single precision
 */
void mpl_bgr2hsv(const Matc3 &bgrImage, Matf3 &hsv);
void mpl_bgr2hsv(const Matc3 &bgrImage, vector<Matf> &hsvf);



cv::Mat dynamicHistogramEqualization(const cv::Mat &bgrImage, const float alpha)
{
	// Work in HSV
	vector<Matf> HSV;
	mpl_bgr2hsv(bgrImage, HSV);
	HSV[0] *= 360.0;
	HSV[1] *= 255.0;

	Matf hist_i, hist_s, hist_c;
	Matf Iscaled = Matf::zeros(bgrImage.size());
	Matc Iint(bgrImage.size());
	Matc Imask = Matc::zeros(Iint.size());

	build_is_histogram(HSV, hist_i, hist_s);
	hist_c = alpha*hist_s + (1-alpha)*hist_i;
	HSV[1] /= 255.0;

	Matd s_r;
	cumulativeSum(hist_c, s_r, true);
	s_r *= 255.0;

	HSV[2].convertTo(Iint, CV_8UC1);
/*
	for (auto n=0; n<255; ++n) {
		Imask = (Iint==n);
		Iscaled.setTo(s_r(n+1,0)/255.0, Imask);
	}
*/

	auto t1=getCurrentTime();
	auto bit = Iscaled.begin();
	for (auto ait=Iint.begin(); ait!=Iint.end(); ++ait) {
		auto av = *ait;
		auto bv = s_r(av+1,0)/255.0;
		*bit = bv;
		++bit;
	}
	auto t2=getCurrentTime();
	cout << "Backproject: " << to_seconds(t2-t1) << endl;

	Iscaled.setTo(1, Iint==255);
	Iscaled *= 255.0;

	cv::Mat bgr, hsv;
	cv::merge(vector<Matf> {HSV[0], HSV[1], Iscaled}, hsv);
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

	cv::Mat bgr8;
	bgr.convertTo(bgr8, CV_8U);
	return bgr8;
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


void build_is_histogram(const vector<Matf> &HSV, Matf &hist_i, Matf &hist_s)
{
	const Matf &I = HSV[2], &S = HSV[1], &H = HSV[0];
	// Channel H is not required
//	H *= 255.0;
	// Assume S has been rescaled
//	S *= 255.0;

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

	Matf Rho = Matf::zeros(I.size());
	auto t1=getCurrentTime();
#pragma omp parallel for
	for (auto r=0; r<Rho.rows; ++r) {
		for (auto c=0; c<Rho.cols; ++c) {
			Matf tmpI, tmpS;
			subMat(I, cv::Point(c,r), tmpI, 5, 5);
			subMat(S, cv::Point(c,r), tmpS, 5, 5);
			flatten(tmpI, tmpI, 0);
			flatten(tmpS, tmpS, 0);
			auto corrv = corrcoeff(tmpI, tmpS);
			auto f = corrv.at<double>(0,1);
			f = fabs(f);
			if (isnan(f)) f=0;
			Rho(r,c) = f;
		}
	}
	auto t2=getCurrentTime();
	cout << "Correlation: " << to_seconds(t2-t1) << endl;

	cv::Mat rd = Rho.mul(dS);
	rd.convertTo(rd, CV_32S);

	hist_i = Matf::zeros(256,1);
	hist_s = Matf::zeros(256,1);

	Mati Intensity;
	t1=getCurrentTime();
	I.convertTo(Intensity, CV_32SC1);
	for (auto n=0; n<255; ++n) {
		cv::Mat temp;
		dIint.copyTo(temp, Intensity==n);
		hist_i(n+1,0) = float(cv::sum(temp).val[0]);
		temp.setTo(0);
		rd.copyTo(temp, Intensity==n);
		hist_s(n+1,0) = float(cv::sum(temp).val[0]);
	}
	t2=getCurrentTime();
	cout << "Histogram: " << to_seconds(t2-t1) << endl;
}


void mpl_bgr2hsv(const Matc3 &bgrImage, vector<Matf> &hsv_split)
{
	hsv_split.clear();

	Matf3 bgrf, hsv1;
	bgrImage.convertTo(bgrf, CV_32F);
	cv::cvtColor(bgrf, hsv1, cv::COLOR_BGR2HSV);

	cv::split(hsv1, hsv_split);
	hsv_split[0] /= 360.0;
}


void mpl_bgr2hsv(const Matc3 &bgrImage, Matf3 &hsv2)
{
	vector<Matf> hsvspl;
	mpl_bgr2hsv(bgrImage, hsvspl);
	cv::merge(hsvspl, hsv2);
}


}		// namespace ice
