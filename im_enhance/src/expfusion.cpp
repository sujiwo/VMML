#include <iostream>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <boost/math/tools/minima.hpp>
#include <Eigen/CholmodSupport>
//#include <eigen3/unsupported/Eigen/SparseExtra>
#include "im_enhance.h"
#include "npy.hpp"


using namespace std;


namespace ice {


template<typename Scalar>
double entropy(const cv::Mat_<Scalar> &X)
{
	assert(X.channels()==1);

	cv::Mat_<Scalar> tmp = X*255;
	tmp.setTo(255, tmp>255);
	tmp.setTo(0, tmp<0);
	Matc tmpd;
	cv::Mat tmpc;
	tmp.convertTo(tmpd, CV_8UC1);
	auto __c = unique(tmpd).getValues();
	auto counts = matFromIterator<float>(__c.begin(), __c.end());
	counts = counts / cv::sum(counts)[0];
	decltype(counts) countsl;
	cv::log(counts, countsl);
	countsl = countsl / log(2);
	return -(cv::sum(counts.mul(countsl))[0]);
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

	cv::resize(imageSmooth, imageSmooth, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
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

	Eigen::CholmodSupernodalLLT<decltype(A)> solver;
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXd out = solver.solve(tin);

	Matf t_vec;
	cv::eigen2cv(out, t_vec);

	Matf Tx = t_vec.reshape(1, imageSmooth.cols).t();
	cv::resize(Tx, Tx, rgbFloat.size(), 0, 0, cv::INTER_CUBIC);
//	tsmooth() is done

	Matb isBad = Tx < 0.5;
//	npy::saveMat(isBad, "/tmp/isBadc.npy"); exit(-1);

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




}	// namespace ice
