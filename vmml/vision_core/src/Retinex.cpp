/*
 * Retinex.cpp
 *
 *  Created on: Feb 10, 2020
 *      Author: sujiwo
 *
 *  XXX: This class is not functional
 */

#include <src/Retinex.h>


using namespace std;


namespace Vmml {


#define INT_PREC 1024.0
#define INT_PREC_BITS 10

inline double int2double(int x) { return (double)x / INT_PREC; }
inline int double2int(double x) { return (int)(x * INT_PREC + 0.5); }

inline int int2smallint(int x) { return (x >> INT_PREC_BITS); }
inline int int2bigint(int x) { return (x << INT_PREC_BITS); }


std::vector<double> CreateKernel(double sigma)
{
	int i, x, filter_size;
	vector<double> filter;
	double sum;

	// Reject unreasonable demands
	if (sigma > 200)
		sigma = 200;

	// get needed filter size (enforce oddness)
	filter_size = (int)floor(sigma*6) / 2;
	filter_size = filter_size * 2 + 1;

	// Allocate kernel space
	filter.resize(filter_size);

	// Calculate exponential
	sum = 0;
	for (i = 0; i < filter_size; i++) {
		x = i - (filter_size / 2);
		filter[i] = exp( -(x*x) / (2*sigma*sigma) );

		sum += filter[i];
	}

	// Normalize
	for (i = 0, x; i < filter_size; i++)
		filter[i] /= sum;

	return filter;
}


std::vector<int> CreateFastKernel(double sigma)
{
	vector<int> kernel;
	int i, filter_size;

	// Reject unreasonable demands
	if ( sigma > 200 ) sigma = 200;

	// get needed filter size (enforce oddness)
	filter_size = (int)floor(sigma*6) / 2;
	filter_size = filter_size * 2 + 1;

	// Allocate kernel space
	kernel.resize(filter_size);

	auto fp_kernel = CreateKernel(sigma);

	for (i = 0; i < filter_size; i++)
		kernel[i] = double2int(fp_kernel[i]);

	return kernel;
}


cv::Mat FilterGaussian(const cv::Mat &img, double sigma)
{
	int i, j, k, source, filter_size;
	cv::Mat temp(img.size(), img.type());
	int v1, v2, v3;

	// Reject unreasonable demands
	if ( sigma > 200 ) sigma = 200;

	// get needed filter size (enforce oddness)
	filter_size = (int)floor(sigma*6) / 2;
	filter_size = filter_size * 2 + 1;

	auto kernel = CreateFastKernel(sigma);

	// filter x axis
	for (j = 0; j < temp.rows; j++)
		for (i = 0; i < temp.cols; i++) {

			// inner loop has been unrolled

			v1 = v2 = v3 = 0;
			for (k = 0; k < filter_size; k++) {

				source = i + filter_size / 2 - k;

				if (source < 0) source *= -1;
				if (source > img.cols - 1) source = 2*(img.cols - 1) - source;

				v1 += kernel[k] * (unsigned char)pc(img, source, j, 0);
				if (img.channels() == 1) continue;
				v2 += kernel[k] * (unsigned char)pc(img, source, j, 1);
				v3 += kernel[k] * (unsigned char)pc(img, source, j, 2);

			}

			// set value and move on
			pc(temp, i, j, 0) = (char)int2smallint(v1);
			if (img->nChannels == 1) continue;
			pc(temp, i, j, 1) = (char)int2smallint(v2);
			pc(temp, i, j, 2) = (char)int2smallint(v3);

		}

	// filter y axis
	for (j = 0; j < img->height; j++)
		for (i = 0; i < img->width; i++) {

			v1 = v2 = v3 = 0;
			for (k = 0; k < filter_size; k++) {

				source = j + filter_size / 2 - k;

				if (source < 0) source *= -1;
				if (source > temp->height - 1) source = 2*(temp->height - 1) - source;

				v1 += kernel[k] * (unsigned char)pc(temp, i, source, 0);
				if (img->nChannels == 1) continue;
				v2 += kernel[k] * (unsigned char)pc(temp, i, source, 1);
				v3 += kernel[k] * (unsigned char)pc(temp, i, source, 2);

			}

			// set value and move on
			pc(img, i, j, 0) = (char)int2smallint(v1);
			if (img->nChannels == 1) continue;
			pc(img, i, j, 1) = (char)int2smallint(v2);
			pc(img, i, j, 2) = (char)int2smallint(v3);

		}
}


cv::Mat FastFilter(cv::Mat &img, double sigma)
{

}


cv::Mat
Retinex (const cv::Mat &img, double sigma, int gain, int offset)
{
	// Initialize temp images
	cv::Mat
		A,
		fA(img.size(), CV_32FC(img.channels())),
		fB(img.size(), CV_32FC(img.channels())),
		fC(img.size(), CV_32FC(img.channels()));

	// Compute log image
	cvConvert( img, fA );
	cvLog( fA, fB );

	// Compute log of blured image
	A = cvCloneImage( img );
	FastFilter( A, sigma );
	cvConvert( A, fA );
	cvLog( fA, fC );

	// Compute difference
	cvSub( fB, fC, fA );

	// Restore
	cvConvertScale( fA, img, gain, offset);

	return A;
}


}	// namespace Vmml
