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

cv::Mat multiScaleRetinexCP(const cv::Mat &rgbImage,
	const float sigma1,
	const float sigma2,
	const float sigma3,
	const float lowClip, const float highClip);
