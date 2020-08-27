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

cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha)
{

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
	npy::saveMat(Az, "/tmp/XY.npy");
	cv::calcCovarMatrix(Az, covar, mean, cv::COVAR_COLS|cv::COVAR_NORMAL);
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
	cov.setTo(-1, cov<-1);
	cov.setTo(1, cov>1);

	return cov;
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
			auto tmpI = subMat(I, cv::Point(c,r), 5, 5);
			auto tmpS = subMat(S, cv::Point(c,r), 5, 5);
			tmpI = flatten(tmpI, 1);
			tmpS = flatten(tmpS, 1);
			auto f = corrcoeff(tmpI, tmpS).at<float>(0,1);
			f = fabsf(f);
			if (isnan(f)) f=0;
			Rho(r,c) = f;
		}
	}

	cv::Mat rd = Rho.mul(dS);
	rd.convertTo(rd, CV_32S);

	hist_i.create(256,1, CV_32FC1);
	hist_s.create(256,1, CV_32FC1);

	Mati Intensity;
	I.convertTo(Intensity, CV_32SC1);
	auto hist_i_m = hist_i.getMat();
	auto hist_s_m = hist_s.getMat();
	for (auto n=0; n<255; ++n) {
		cv::Mat temp = Mati::zeros(dIint.size());
		dIint.copyTo(temp, Intensity==n);
		hist_i_m.at<double>(n+1) = cv::sum(temp).val[0];
		temp.setTo(0);
		rd.copyTo(temp, Intensity==n);
		hist_s_m.at<double>(n+1) = cv::sum(temp).val[0];
	}

	// Done?
}


}		// namespace ice
