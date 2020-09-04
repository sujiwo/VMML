#ifndef __MATUTILS_H__
#define __MATUTILS_H__ 1

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <opencv2/core/eigen.hpp>


namespace ice {

typedef cv::Mat_<float> Matf;
typedef cv::Mat_<double> Matd;
typedef cv::Mat_<cv::Vec3f> Matf3;
typedef cv::Mat_<int> Mati;
// Unsigned 32-bit Integer is not supported by OpenCV
typedef cv::Mat_<uint> Matui;
typedef cv::Mat_<bool> Matb;
typedef cv::Mat_<unsigned char> Matc;
typedef cv::Mat_<cv::Vec3b> Matc3;


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
void flatten(const cv::Mat_<Scalar> &in, cv::Mat_<Scalar> &out, uchar order=0)
{
	if (order==0) {
		out = in.reshape(0, in.rows*in.cols);
	}
	else if (order==1) {
		out.create(in.rows*in.cols, 1);
		for (int c=0; c<in.cols; ++c) {
			in.col(c).copyTo(out.rowRange(c*in.rows, c*in.rows+in.rows));
		}
	}
}


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
 * Create Vector from iterator
 */
template<typename Scalar, class InputIterator>
cv::Mat_<Scalar>
matFromIterator (
	InputIterator begin,
	InputIterator end)
{
	std::vector<Scalar> v;
	for (auto b=begin; b!=end; ++b) {
		v.push_back(*b);
	}

	cv::Mat_<Scalar> _c(v.size(), 1, v.data());
	return _c.clone();
}


template<typename K, typename V>
std::vector<K>
getKeys (const std::map<K, V> &M)
{
	std::vector<K> keys;
	for (auto &p: M) {
		keys.push_back(p.first);
	}
	return keys;
}

template<typename K, typename V>
class MapFun: public std::map<K, V>
{
public:
	std::vector<K> getKeys()
	{
		std::vector<K> keys;
		for (auto &p: *this) {
			keys.push_back(p.first);
		}
		return keys;
	}

	std::vector<V> getValues()
	{
		std::vector<V> vals;
		for (auto &p: *this) {
			vals.push_back(p.second);
		}
		return vals;
	}
};


template<typename Scalar>
MapFun<Scalar, int> unique(const cv::Mat_<Scalar> &M)
{
	MapFun<Scalar,int> res;
	for (auto mit=M.begin(); mit!=M.end(); ++mit) {
		auto m = *mit;
		if (res.find(m)==res.end()) {
			res[m] = 1;
		}
		else res[m] += 1;
	}

	return res;
}


template<typename Scalar>
cv::Mat_<Scalar> applyK(const cv::Mat_<Scalar> &input, float k, float a, float b)
{
	auto beta = exp((1-pow(k,a))*b);
	auto gamma = pow(k, a);
	cv::Mat_<Scalar> _powf;
	cv::pow(input, gamma, _powf);
	return _powf * beta;
}


template<typename Scalar>
double entropy(const cv::Mat_<Scalar> &X)
{
	assert(X.channels()==1);

	cv::Mat_<Scalar> tmp = X*255;
	tmp.setTo(255, tmp>255);
	tmp.setTo(0, tmp<0);
	Matc tmpd;
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
 * Create a submatrix from input with a center point specified in anchor.
 * If anchor is nearby input border, apply border padding.
 */
template<typename Scalar>
void subMat(const cv::Mat_<Scalar> &input,
	const cv::Point &anchor,
	cv::Mat_<Scalar> &output,
	int width=-1, int height=-1,
	int borderType=cv::BORDER_REPLICATE, const cv::Scalar &value=cv::Scalar())
{
	if (width==-1)
		width = output.cols;
	if (height==-1)
		height = output.rows;

	int top_left_x = anchor.x-width/2;
	int top_left_y = anchor.y-width/2;
	int bottom_right_x = top_left_x + width;
	int bottom_right_y = top_left_y + height;

	if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows) {
		// border padding will be required
		int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

		if (top_left_x < 0) {
			width = width + top_left_x;
			border_left = -1 * top_left_x;
			top_left_x = 0;
		}
		if (top_left_y < 0) {
			height = height + top_left_y;
			border_top = -1 * top_left_y;
			top_left_y = 0;
		}
		if (bottom_right_x > input.cols) {
			width = width - (bottom_right_x - input.cols);
			border_right = bottom_right_x - input.cols;
		}
		if (bottom_right_y > input.rows) {
			height = height - (bottom_right_y - input.rows);
			border_bottom = bottom_right_y - input.rows;
		}

		cv::Rect R(top_left_x, top_left_y, width, height);
		copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, borderType, value);
	}
	else {
		// no border padding required
		cv::Rect R(top_left_x, top_left_y, width, height);
		input(R).copyTo(output);
	}
}



template<typename Scalar>
void cumulativeSum(const cv::Mat_<Scalar> &input, Matd& accum, bool normalized=false)
{
	assert(input.channels()==1);

	if (accum.size()!=input.size()) {
		accum.create(input.size());
	}
	accum.setTo(0);

	auto r=1;
	for (auto ita=input.begin(), itb=accum.begin(); ita!=input.end(); ita++, itb++) {
		*itb = *(itb-1) + *ita;
		++r;
	}

	if (normalized) {
		double nm = cv::sum(input)[0];
		accum /= nm;
	}
}


Matf cdf (cv::Mat &grayImage, cv::Mat mask, bool normalized=true);


}		// namespace ice

#endif	// __MATUTILS_H__
