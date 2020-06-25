#include <opencv2/core.hpp>


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
