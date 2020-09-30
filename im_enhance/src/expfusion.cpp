#include <iostream>
#include <vector>
#include <type_traits>
#include <opencv2/imgproc.hpp>
#include <boost/math/tools/minima.hpp>
//#include <Eigen/SparseCholesky>
#include <Eigen/CholmodSupport>
//#include "MUMPSSupport"
#include <unsupported/Eigen/SparseExtra>
#include "im_enhance.h"
#include "matutils.h"
#include "timer.h"
#include "npy.hpp"


using namespace std;


namespace ice {


template<typename Scalar>
cv::Mat_<Scalar>
selectElementsToVectorWithMask(const cv::Mat_<Scalar> &input, cv::InputArray mask_)
{
	assert(input.channels()==1);
	assert(input.size()==mask_.size());
	auto mask = mask_.getMat();

	std::vector<Scalar> V;
	for (int r=0; r<input.rows; ++r)
		for (int c=0; c<input.cols; ++c) {
			auto m = mask.at<int>(r,c);
			if (m!=0)
				V.push_back(input(r,c));
		}
	return cv::Mat_<Scalar>(V.size(), 1, V.data());
}


Eigen::SparseMatrix<float>
spdiags(const Matf &_Data, const Mati &_diags, int m, int n)
{
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Data;
	Eigen::VectorXi diags;
	cv::cv2eigen(_Data, Data);
	cv::cv2eigen(_diags, diags);
	return spdiags(Data, diags, m, n);
}


/*
template<typename SrcScalar, typename NewScalar=SrcScalar>
Eigen::SparseMatrix<NewScalar> spdiags(const cv::Mat_<SrcScalar> &Data, const std::vector<int> &diags, int m, int n)
{
	assert(Data.rows==diags.size());
	Eigen::Matrix<NewScalar, Eigen::Dynamic, Eigen::Dynamic> Data_;
	cv::cv2eigen(Data, Data_);
	Eigen::VectorXi diags_(diags.size());
	for (int i=0; i<diags.size(); ++i)
		diags[i] = _diags[i];
	return spdiags(Data_, diags_, m, n);
}
*/


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


template<typename DstScalar, typename SrcScalar>
Eigen::SparseMatrix<DstScalar>
spdiags(const std::vector<cv::Mat_<SrcScalar>> &_Data, const std::vector<int> &diags, int m, int n)
{
	std::vector<Eigen::Triplet<DstScalar>> triplets;
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

	Eigen::SparseMatrix<DstScalar> A(m, n);
	A.setFromTriplets(triplets.begin(), triplets.end());

	return A;
}


/*
 * Create sparse diagonal matrix using single vector
 */
template<typename DstScalar, typename SrcScalar>
Eigen::SparseMatrix<DstScalar>
spdiags(cv::Mat_<SrcScalar> &_Data, int m, int n)
{
	std::vector<Eigen::Triplet<DstScalar>> triplets;
	int diagonalLength = std::min(m, n);
	assert (_Data.rows * _Data.cols == diagonalLength);
	triplets.reserve(diagonalLength);

	auto vit = _Data.begin();
	for (int i=0; i<diagonalLength; ++i, ++vit) {
		triplets.push_back( {i, i, *vit} );
	}

	Eigen::SparseMatrix<DstScalar> A(m, n);
	A.setFromTriplets(triplets.begin(), triplets.end());

	return A;
}


template<typename SrcScalar>
Eigen::SparseMatrix<SrcScalar>
spdiags(const std::vector<cv::Mat_<SrcScalar>> &_Data, const std::vector<int> &diags, int m, int n)
{
	return spdiags<SrcScalar, SrcScalar>(_Data, diags, m, n);
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

	const float resizeWorkFactor = 0.3333333333333;
	cv::resize(imageSmooth, imageSmooth, cv::Size(), resizeWorkFactor, resizeWorkFactor, cv::INTER_CUBIC);
	cv::normalize(imageSmooth, imageSmooth, 0.0, 1.0, cv::NORM_MINMAX);

	// computeTextureWeights()
	// Calculate gradient (horizontal & vertical)
	Matf dt0v, dt0h, gh, gv, Wh, Wv;
	const int ksize = 3;
	cv::Sobel(imageSmooth, dt0v, -1, 0, 1, ksize);
	cv::Sobel(imageSmooth, dt0h, -1, 1, 0, ksize);

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

	/*
	 * Note: Cholmod requires input matrices in double precision
	 */
	vector<Matf> Aconc = {dxd1, dxd2};
	vector<int> Adgl = {-k+Wh.rows, -Wh.rows};
	auto Ax = spdiags<double>(Aconc, Adgl, k, k);

	Aconc = {dyd1, dyd2};
	Adgl = {-Wv.rows+1, -1};
	auto Ay = spdiags<double>(Aconc, Adgl, k, k);

	Matf D = 1 - (dx + dy + dxa + dya);
	decltype(Ax) Axyt = (Ax+Ay);
	Axyt = Axyt.conjugate().transpose();

	auto Dspt = spdiags<double>(D, k, k);
	Eigen::SparseMatrix<double> A = (Ax+Ay) + Axyt + Dspt;

	auto _tin = flatten(imageSmooth, 1);
	Eigen::Matrix<double,-1,-1> tin;
	cv::cv2eigen(_tin, tin);

/*
	Eigen::SimplicialLDLT<decltype(A)> solver;
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXd out = solver.solve(tin);
*/

	Eigen::CholmodSupernodalLLT<decltype(A), Eigen::Upper> solver;
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXd out = solver.solve(tin);

/*
	Eigen::MUMPSLDLT<decltype(A), Eigen::Upper> solver;
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXd out = solver.solve(tin);
*/

	Matf t_vec;
	cv::eigen2cv(out, t_vec);

	Matf Tx = t_vec.reshape(1, imageSmooth.cols).t();
	cv::resize(Tx, Tx, rgbFloat.size(), 0, 0, cv::INTER_CUBIC);
//	tsmooth() is done

	Matb isBad = Tx < 0.5;

	/* Maximize entrophy */
	Matf3 rgbTiny;
	cv::resize(rgbFloat, rgbTiny, cv::Size(50,50), 0, 0, cv::INTER_AREA);

	Matf isBadf;
	isBad.convertTo(isBadf, CV_32F);
	cv::normalize(isBadf, isBadf, 0.0, 1.0, cv::NORM_MINMAX);
	cv::resize(isBadf, isBadf, cv::Size(50,50), 0, 0, cv::INTER_CUBIC);

	isBadf.setTo(0, isBadf<0.5);
	isBadf.setTo(1, isBadf>=0.5);

	// rgb2gm
	cv::normalize(rgbTiny, rgbTiny, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
	vector<float> Yv;
	Matf Y(rgbTiny.size());
	for (int r=0; r<rgbTiny.rows; ++r) {
		for (int c=0; c<rgbTiny.cols; ++c) {
			auto &ch = rgbTiny(r, c);
			if (ch[0]<0) ch[0]=0;
			if (ch[1]<0) ch[1]=0;
			if (ch[2]<0) ch[2]=0;
			Y(r,c) = fabsf(cbrtf(ch[0]*ch[1]*ch[2]));
			if (isBadf(r,c)!=0)
				Yv.push_back(Y(r,c));
		}
	}

	Matf Yx(Yv.size(), 1, Yv.data());

	// What to do for bad vector?
	if (Yx.rows*Yx.cols==0) {
	}

	// define functions
	auto funEntropy = [&](float k)->double {
		return -entropy(applyK(Yx, k, a_, b_));
	};

	// XXX: Boost's Brent Method implementation is different from Numpy
	auto fmin = boost::math::tools::brent_find_minima(funEntropy, 1.0, 7.0, numeric_limits<double>::digits10);
	auto J = applyK(rgbFloat, fmin.first, a_, b_) - 0.01;

	// Combine Tx
	Matf3 T_all;
	cv::merge(std::vector<Matf>{Tx,Tx,Tx}, T_all);
	cv::pow(T_all, lambda, T_all);

	Matf3 I2 = rgbFloat.mul(T_all);
	Matf3 J2 = J.mul(cv::Vec3f(1,1,1)-T_all);

	T_all.release();
	Matf3 result = (I2 + J2)*255;

	for (auto &px: result) {
		px[0] = (px[0]>255 ? 255 : (px[0]<0 ? 0 : px[0]));
		px[1] = (px[1]>255 ? 255 : (px[1]<0 ? 0 : px[1]));
		px[2] = (px[2]>255 ? 255 : (px[2]<0 ? 0 : px[2]));
	}

	cv::Mat Outf;
	result.convertTo(Outf, CV_8UC(3));

	return Outf;
}




}	// namespace ice
