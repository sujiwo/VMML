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
 * Matrix utilities
 */
void shiftCol(cv::Mat &in, cv::Mat &out, int numToRight=0);
void shiftRow(cv::Mat &in, cv::Mat &out, int numToBelow=0);

cv::Mat shiftCol(cv::Mat &in, int numToRight=0)
{
	cv::Mat out;
	shiftCol(in, out, numToRight);
	return out;
}

cv::Mat shiftRow(cv::Mat &in, int numToBelow=0)
{
	cv::Mat out;
	shiftRow(in, out, numToBelow);
	return out;
}

/*
 * Flatten an array into one dimensional
 * Order: 0 => row-major
 *        1 => column-major
 */
cv::Mat flatten(cv::InputArray src, uchar order=0);

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
void spdiags(const cv::Mat &_Data,
	const cv::Mat &diags, int m, int n,
	Eigen::SparseMatrix<float>& dst);




