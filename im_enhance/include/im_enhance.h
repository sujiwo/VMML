#include <exception>
#include <algorithm>
#include <iterator>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <opencv2/core/eigen.hpp>


namespace ice {





cv::Mat autoAdjustGammaRGB (const cv::Mat &rgbImg, cv::InputArray mask=cv::noArray());

cv::Mat toIlluminatiInvariant (const cv::Mat &imageRgb, const float alpha);

/*
 * Retinex Family
 *
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
	const float highClip=0.99);

/*
 * Dynamic Histogram Equalization (choice #3)
 */
cv::Mat dynamicHistogramEqualization(const cv::Mat &rgbImage, const float alpha=0.5);

/*
 * Exposure Fusion (choice #4)
 */
cv::Mat exposureFusion(const cv::Mat &rgbImage);


// Testing only, need to used only internally
cv::Mat matlab_covar(const cv::Mat &A, const cv::Mat &B);
cv::Mat corrcoeff (const cv::Mat &M1, const cv::Mat &M2);


}		// namespace ice



