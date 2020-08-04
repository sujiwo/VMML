#include <vector>
#include <array>
#include <iostream>
#include <exception>
#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <boost/math/tools/minima.hpp>
#include <eigen3/unsupported/Eigen/SparseExtra>
#include "im_enhance.h"
#include "npy.hpp"


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
	rgbBuf[0] = setGamma (rgbBuf[0], gamma);
	rgbBuf[1] = setGamma (rgbBuf[1], gamma);
	rgbBuf[2] = setGamma (rgbBuf[2], gamma);

	cv::Mat BGRres;
	cv::merge (rgbBuf, BGRres);
	return BGRres;
}


cv::Mat toIlluminatiInvariant (const Matc3 &rgbImage, const float alpha)
{
	Matc iImage (rgbImage.rows, rgbImage.cols);

	auto oImgIt = iImage.begin();
	for (auto it=rgbImage.begin(); it!=rgbImage.end(); ++it) {
		float
			fb = (*it)[0] / 255.0,
			fg = (*it)[1] / 255.0,
			fr = (*it)[2] / 255.0;
		float iPix = 0.5 + logf(fg) - alpha*logf(fb) - (1-alpha)*logf(fr);
		(*oImgIt) = (uchar)(iPix*255);
		oImgIt++;
	}

	return iImage;
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
	cv::multiply(cv::Mat::ones(inp.size(), CV_32FC1), inpLog, inpLog, 1/log(10));

	// GaussianBlur() is also a hotspot for large sigma
	cv::Mat gaussBlur;
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);

	cv::log(gaussBlur, gaussBlur);
	cv::multiply(cv::Mat::ones(inp.size(), CV_32FC1), gaussBlur, gaussBlur, 1/log(10));

	cv::Mat R;
	cv::subtract(inpLog, gaussBlur, R);
	return R;
}


cv::UMat
singleScaleRetinex(const cv::UMat &inp, const float sigma)
{
	assert(inp.type()==CV_32FC1);

	// XXX: log_e or log_10 ?
	cv::UMat inpLog;
	cv::log(inp, inpLog);
	cv::multiply(cv::UMat::ones(inp.size(), CV_32FC1), inpLog, inpLog, 1/log(10));

	cv::UMat gaussBlur;
	cv::GaussianBlur(inp, gaussBlur, cv::Size(0,0), sigma);

	cv::log(gaussBlur, gaussBlur);
	cv::multiply(cv::UMat::ones(inp.size(), CV_32FC1), gaussBlur, gaussBlur, 1/log(10));

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
		cv::UMat ssRetx = singleScaleRetinex(inputg, s);
		cv::add(msrex, ssRetx, msrex);
	}

	cv::divide(3, msrex, msrex);

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
 * Order: 0 => row-major
 *        1 => column-major
 *
 */
/*
cv::Mat flatten(cv::InputArray src, uchar order)
{
	cv::Mat out = cv::Mat::zeros(src.cols()*src.rows(), 1, src.type());
	cv::Mat in = src.getMat();

	// Row-major
	if (order==0) {
		for (int r=0; r<in.rows; ++r) {
			out.rowRange(r*in.cols, r*in.cols+in.cols) = in.row(r).t();
		}
	}
	else if (order==1) {
		for (int c=0; c<in.cols; ++c) {
			in.col(c).copyTo(out.rowRange(c*in.rows, c*in.rows+in.rows));
		}
	}
	else throw runtime_error("Unsupported order");

	return out;
}


cv::Mat reshape(cv::InputArray _src, int row, int col, uchar order)
{
	assert ((_src.cols()==1 or _src.rows()==1) and _src.cols()*_src.rows()==row*col);

	cv::Mat
		src = _src.getMat(),
		dst(row, col, _src.type());
	if (_src.cols()==1) src = src.t();

	if (order==0) {
		for (int i=0; i<row; ++i) {
			auto S = src.rowRange(i*col, (i+1)*col);

		}
	}

	else if (order==1) {
	}

	else throw runtime_error("Unsupported order");

	return dst;
}
*/


Eigen::SparseMatrix<float>
spdiags(const Matf &_Data, const Mati &_diags, int m, int n)
{
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Data;
	Eigen::VectorXi diags;
	cv::cv2eigen(_Data, Data);
	cv::cv2eigen(_diags, diags);
	return spdiags(Data, diags, m, n);
}


Eigen::SparseMatrix<float>
spdiags(const Matf &_Data, const std::vector<int> &_diags, int m, int n)
{
	assert(_Data.rows==_diags.size());
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Data;
	cv::cv2eigen(_Data, Data);
	Eigen::VectorXi diags(_diags.size());
	for (int i=0; i<_diags.size(); ++i)
		diags[i] = _diags[i];
	return spdiags(Data, diags, m, n);
}


template<typename Scalar>
Eigen::SparseMatrix<Scalar>
spdiags(const std::vector<cv::Mat_<Scalar>> &_Data, const std::vector<int> &diags, int m, int n)
{
	std::vector<Eigen::Triplet<float>> triplets;
	triplets.reserve(std::min(m,n)*diags.size());

	for (int k = 0; k < diags.size(); ++k) {
		auto Data = _Data[k];
		assert(Data.cols==_Data[0].cols && Data.rows==_Data[0].rows);
		int diag = diags[k];	// get diagonal
		int i_start = std::max(-diag, 0); // get row of 1st element
		int i_end = std::min(m, m-diag-(m-n)); // get row of last element
		int j = -std::min(0, -diag); // get col of 1st element
		int B_i; // start index i in matrix B
		if(m < n)
			B_i = std::max(-diag,0); // m < n
		else
			B_i = std::max(0,diag); // m >= n
		for(int i = i_start; i < i_end; ++i, ++j, ++B_i){
			triplets.push_back( {i, j,  *Data[B_i]} );
		}
	}

	Eigen::SparseMatrix<Scalar> A(m, n);
	A.setFromTriplets(triplets.begin(), triplets.end());

	return A;
}


/*
 * Raises A(x) to power from B(x)
 */
void MatPow(Matf3 &A, const Matf &B)
{
	assert(A.size()==B.size());

	auto bit = B.begin();
	for (auto ait=A.begin(); ait!=A.end(); ++ait, ++bit) {
		(*ait)[0] = pow((*ait)[0], *bit);
		(*ait)[1] = pow((*ait)[1], *bit);
		(*ait)[2] = pow((*ait)[2], *bit);
	}
}


/*
Matb operator < (const Matf &inp, const float X)
{
	Matb B(inp.size());

	auto inpIt = inp.begin();
	auto BIt = B.begin();
	for (; inpIt!=inp.end(); ++inpIt, ++BIt) {
		*BIt = *inpIt < X;
	}

	return B;
}
*/


/*
Matb operator > (const Matf &inp, const float X)
{
	Matb B(inp.size());

	auto inpIt = inp.begin();
	auto BIt = B.begin();
	for (; inpIt!=inp.end(); ++inpIt, ++BIt) {
		*BIt = *inpIt > X;
	}

	return B;
}
*/


void saveMatNp(const cv::Mat &M, const char *filename)
{
	return saveMatNp(M, filename);
}


/*
 * Ying Et Al
 */
// Default parameters
const float sharpness=1e-3;
const int sigma = 5;
const auto lambda = 0.5;
const float
	a_ = -0.3293,
	b_ = 1.1258;

cv::Mat exposureFusion(const cv::Mat &rgbImage)
{
	Matf L, imageSmooth(rgbImage.size());
	Matf3 rgbFloat;

	cv::normalize(rgbImage, rgbFloat, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);

	for (uint r=0; r<rgbFloat.rows; ++r) {
		for (uint c=0; c<rgbFloat.cols; ++c) {
			auto vc = rgbFloat(r,c);
			imageSmooth(r,c) = max(vc[0], max(vc[1], vc[2]));
		}
	}

	// XXX: There's difference between OpenCV's Bicubic vs PIL
	cv::resize(imageSmooth, imageSmooth, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
	cv::normalize(imageSmooth, imageSmooth, 0.0, 1.0, cv::NORM_MINMAX);

	// computeTextureWeights()
	// Calculate gradient (horizontal & vertical)
	Matf dt0v, dt0h, gh, gv, Wh, Wv;
	cv::Sobel(imageSmooth, dt0v, -1, 0, 1, 1);
	cv::Sobel(imageSmooth, dt0h, -1, 1, 0, 1);

	cv::filter2D(dt0v, gv, -1, cv::Mat::ones(sigma, 1, CV_64F), cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);
	cv::filter2D(dt0h, gh, -1, cv::Mat::ones(1, sigma, CV_64F), cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);

	Wh = 1.0f / (cv::abs(gh).mul(cv::abs(dt0h)) + sharpness);  // wx
	Wv = 1.0f / (cv::abs(gv).mul(cv::abs(dt0v)) + sharpness);  // wy

	// Solving linear equation for T (in vectorized form)
	auto k = imageSmooth.rows * imageSmooth.cols;
	auto dx = -lambda * flatten(Wh, 1);
	auto dy = -lambda * flatten(Wv, 1);
	auto tempx = shiftCol(Wh, 1);
	auto tempy = shiftRow(Wv, 1);
	Matf dxa = -lambda * flatten(tempx, 1);
	Matf dya = -lambda * flatten(tempy, 1);

	auto tmp = Wh.col(Wh.cols-1);
	cv::hconcat(tmp, cv::Mat::zeros(Wh.rows, Wh.cols-1, Wh.type()), tempx);
	tmp = Wv.row(Wv.rows-1);
	cv::vconcat(tmp, cv::Mat::zeros(Wv.rows-1, Wv.cols, Wv.type()), tempy);
	Matf dxd1 = -lambda * flatten(tempx, 1);
	Matf dyd1 = -lambda * flatten(tempy, 1);

	Wh.col(Wh.cols-1) = 0;
	Wv.row(Wv.rows-1) = 0;
	auto dxd2 = -lambda * flatten(Wh, 1);
	auto dyd2 = -lambda * flatten(Wv, 1);

	vector<Matf> Aconc = {dxd1, dxd2};
	vector<int> Adgl = {-k+Wh.rows, -Wh.rows};
	auto Ax = spdiags(Aconc, Adgl, k, k);

	Aconc = {dyd1, dyd2};
	Adgl = {-Wv.rows+1, -1};
	auto Ay = spdiags(Aconc, Adgl, k, k);

	Matf D = 1 - (dx + dy + dxa + dya);
	decltype(Ax) Axyt = (Ax+Ay);
	Axyt = Axyt.conjugate().transpose();

	decltype(Ay) Dspt = spdiags(D.t(), vector<int>{0}, k, k).transpose();
	decltype(Ax) A = (Ax+Ay) + Axyt + Dspt;

	// Debugging
	Eigen::saveMarket(A, "/tmp/Asparse");
	cout << "A saved\n";
	exit(1);

	auto _tin = flatten(imageSmooth, 1);
	Eigen::Matrix<float,-1,-1> tin;
	cv::cv2eigen(_tin, tin);

	Eigen::SparseLU<decltype(A)> solver;
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXf out = solver.solve(tin);

	Matf t_vec;
	cv::eigen2cv(out, t_vec);

	Matf Tx = t_vec.reshape(1, imageSmooth.cols).t();
	/*
	 * XXX: The Tx produced by ours is signficantly different from Python Version.
	 * Need more verification on the steps
	 */
	cv::resize(Tx, Tx, rgbFloat.size(), 0, 0, cv::INTER_CUBIC);
//	tsmooth() is done

	Matc isBad = Tx < 0.5;

	/* Maximize entrophy */
	Matf3 rgbTiny;
	cv::resize(rgbFloat, rgbTiny, cv::Size(50,50), 0, 0, cv::INTER_AREA);

	// rgb2gm
	cv::normalize(rgbTiny, rgbTiny, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
	Matf Y(rgbTiny.size());
	for (int r=0; r<rgbTiny.rows; ++r) {
		for (int c=0; c<rgbTiny.cols; ++c) {
			auto ch = rgbTiny(r, c);
			Y(r,c) = fabsf(cbrtf(ch[0]*ch[1]*ch[2]));
		}
	}

	Matf isBadf;
	isBad.convertTo(isBadf, CV_32FC1);
	cv::resize(isBadf, isBadf, cv::Size(50,50), 0, 0, cv::INTER_CUBIC);

	isBadf.setTo(0, isBadf<0.5);
	isBadf.setTo(1, isBadf>=0.5);
	auto Yx = selectElementsToVectorWithMask(Y, cv::Mat(isBadf==1));

	// What to do for bad vector?
	if (Yx.rows*Yx.cols==0) {

	}

	// define functions
	auto funEntropy = [&](float k)->float {
		return -entropy(applyK(Yx, k, a_, b_));
	};

	// call Boost Brent Method
	auto fmin = boost::math::tools::brent_find_minima(funEntropy, 1.0, 7.0, numeric_limits<float>::digits);
	auto J = applyK(rgbFloat, fmin.first, a_, b_) - 0.01;

	// Combine Tx
	Matf3 T_all;
	cv::merge(std::vector<Matf>{Tx,Tx,Tx}, T_all);
	cv::pow(T_all, lambda, T_all);

	Matf3 I2 = rgbFloat.mul(T_all);
	Matf3 J2 = J.mul(1-T_all);

	T_all.release();
	Matf3 result = (I2 + J2)*255;

	for (auto &px: result) {
		px[0] = (px[0]>255 ? 255 : (px[0]<0 ? 0 : px[0]));
		px[1] = (px[1]>255 ? 255 : (px[1]<0 ? 0 : px[1]));
		px[2] = (px[2]>255 ? 255 : (px[2]<0 ? 0 : px[2]));
	}
/*
	result.setTo(255, result>255);
	result.setTo(0, result<0);
*/

	cv::Mat Outf;
	result.convertTo(Outf, CV_8U);

	return Outf;
}

