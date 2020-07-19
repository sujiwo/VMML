#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <opencv2/core/eigen.hpp>


cv::Mat autoAdjustGammaRGB (const cv::Mat &rgbImg, cv::InputArray mask=cv::noArray());

/*
 * Retinex Family
 */
static cv::Mat
singleScaleRetinex(const cv::Mat &inp, const float sigma);

static cv::Mat
multiScaleRetinex(const cv::Mat &inp,
	const float sigma1,
	const float sigma2,
	const float sigma3);

static cv::Mat
simpleColorBalance(const cv::Mat &inp, const float lowClip, const float highClip);

/*
 * Suggested values:
 * Sigmas = { 15, 80, 250 }
 * low clip = 0.01
 * high clip = 0.9999999
 */

cv::Mat multiScaleRetinexCP(const cv::Mat &rgbImage,
	const float sigma1=15.0,
	const float sigma2=80.0,
	const float sigma3=250.0,
	const float lowClip=0.01,
	const float highClip=0.99999999999);

/*
 * Dynamic Histogram Equalization (choice #3)
 */
cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha=0.5);

/*
 * Exposure Fusion (choice #4)
 */
cv::Mat exposureFusion(const cv::Mat &rgbImage);


typedef cv::Mat_<float> Matf;
typedef cv::Mat_<cv::Vec3f> Matf3;
typedef cv::Mat_<int> Mati;
typedef cv::Mat_<uint> Matui;
typedef cv::Mat_<bool> Matb;

/*
 * Matrix utilities
 */

template<typename Scalar>
void shiftCol(cv::Mat_<Scalar> &in, cv::Mat_<Scalar> &out, int numToRight=0)
{
	if (numToRight==0) {
		in.copyTo(out);
		return;
	}

	out.create(in.size());
	numToRight = numToRight % in.cols;
	if (numToRight<0)
		numToRight = in.cols + numToRight;

	in(cv::Rect(in.cols-numToRight,0, numToRight,in.rows)).copyTo(out(cv::Rect(0,0,numToRight,in.rows)));
	in(cv::Rect(0,0, in.cols-numToRight,in.rows)).copyTo(out(cv::Rect(numToRight,0,in.cols-numToRight,in.rows)));
}

template<typename Scalar>
void shiftRow(cv::Mat_<Scalar> &in, cv::Mat_<Scalar> &out, int numToBelow)
{
	if (numToBelow==0) {
		in.copyTo(out);
		return;
	}

	out.create(in.size());
	numToBelow = numToBelow % in.rows;
	if (numToBelow<0)
		numToBelow = in.rows + numToBelow;

	in(cv::Rect(0,in.rows-numToBelow, in.cols, numToBelow)).copyTo(out(cv::Rect(0,0, in.cols,numToBelow)));
	in(cv::Rect(0,0, in.cols,in.rows-numToBelow)).copyTo(out(cv::Rect(0,numToBelow, in.cols,in.rows-numToBelow)));
}

template<typename Scalar>
cv::Mat_<Scalar> shiftCol(cv::Mat_<Scalar> &in, int numToRight=0)
{
	cv::Mat_<Scalar> out;
	shiftCol(in, out, numToRight);
	return out;
}

template<typename Scalar>
cv::Mat_<Scalar> shiftRow(cv::Mat_<Scalar> &in, int numToBelow=0)
{
	cv::Mat_<Scalar> out;
	shiftRow(in, out, numToBelow);
	return out;
}

/*
 * Flatten an array into one dimensional
 * Order: 0 => row-major
 *        1 => column-major
 */
//cv::Mat flatten(cv::InputArray src, uchar order=0);

template<typename Scalar>
cv::Mat_<Scalar>
flatten(const cv::Mat_<Scalar> &in, uchar order=0)
{
	cv::Mat_<Scalar> out = cv::Mat_<Scalar>::zeros(in.rows*in.cols, 1);
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
	else throw std::runtime_error("Unsupported order");

	return out;
}

/*
 * Inverse of the above function:
 */
/*
cv::Mat reshape(cv::InputArray src, int row, int col, uchar order=0);

template<typename Scalar>
cv::Mat_<Scalar>
reshape(const cv::Mat_<Scalar> &in, const int row, const int col, uchar order=0)
{
	assert ((in.cols()==1 or in.rows()==1) and in.cols()*in.rows()==row*col);

	cv::Mat_<Scalar> dst(row, col);
	cv::Mat_<Scalar> src;
	if (src.cols==1) src = in.t();
	else src = in;


}
*/


/*
 * Emulating `spdiags' from Scipy
 * Data: matrix diagonals stored row-wise
 */
template<typename Derived>
Eigen::SparseMatrix<typename Derived::Scalar>
spdiags(const Eigen::MatrixBase<Derived> &Data, const Eigen::VectorXi &diags, int m, int n)
{
	typedef Eigen::Triplet<typename Derived::Scalar> triplet_t;
	std::vector<triplet_t> triplets;
	triplets.reserve(std::min(m,n)*diags.size());

	for (int k = 0; k < diags.size(); ++k) {
		int diag = diags(k);	// get diagonal
		int i_start = std::max(-diag, 0); // get row of 1st element
		int i_end = std::min(m, m-diag-(m-n)); // get row of last element
		int j = -std::min(0, -diag); // get col of 1st element
		int B_i; // start index i in matrix B
		if(m < n)
			B_i = std::max(-diag,0); // m < n
		else
			B_i = std::max(0,diag); // m >= n
		for(int i = i_start; i < i_end; ++i, ++j, ++B_i){
			triplets.push_back( {i, j,  Data(k,B_i)} );
		}
	}

	Eigen::SparseMatrix<typename Derived::Scalar> A(m, n);
	A.setFromTriplets(triplets.begin(), triplets.end());

	return A;
}


/*
 * Same as above, using Opencv Input
 */
/*
Eigen::SparseMatrix<float>
spdiags(const cv::Mat &_Data,
	const cv::Mat &diags, int m, int n);

Eigen::SparseMatrix<float>
spdiags(const std::vector<cv::Mat> &Data, const std::vector<int> &diags, int m, int n);

Eigen::SparseMatrix<float>
spdiags(const cv::Mat &_Data, const std::vector<int> &diags, int m, int n);
*/