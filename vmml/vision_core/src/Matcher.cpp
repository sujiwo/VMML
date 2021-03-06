/*
 * Matcher.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: sujiwo
 */

#include <omp.h>
#include <algorithm>
#include <limits>
#include <array>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/ml.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "vmml/Matcher.h"
#include "vmml/VisionMap.h"
#include "vmml/Triangulation.h"
#include "vmml/MapPoint.h"


using namespace std;
using namespace Eigen;


namespace Vmml {


typedef
	std::map<kpid, std::set<kpid>>
		WhichKpId;

float
	Matcher::circleOfConfusionDiameter = 4.0;

const cv::Scalar
	colorBlue(255, 0, 0),
	colorGreen(0, 255, 0),
	colorRed(0, 0, 255),
	colorYellow(0, 255, 255);
const int
	pointRadius = 3;
int
	Matcher::__maxDraw = 1;

/*
 * Parameters for Optical Flow
 */
const cv::Size optFlowWindowSize(15, 15);
const int maxLevel = 2;
const cv::TermCriteria optFlowStopCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT, 10, 0.03);


int countFeaturePairMask(const cv::Mat &M)
{
	assert (M.cols==1 and M.type()==CV_8UC1);

	int s=0;
	for (int r=0; r<M.rows; ++r) {
		if (M.at<char>(r,0)!=0)
			s+=1;
	}

	return s;
}


void filterFeaturePairMask(std::vector<Matcher::KpPair> &srcFeaturePair, const cv::Mat &M)
{
	assert (M.rows==srcFeaturePair.size());
	vector<Matcher::KpPair> newPairList;

	for (int i=0; i<srcFeaturePair.size(); ++i) {
		if (M.at<char>(i,0)!=0)
			newPairList.push_back(srcFeaturePair.at(i));
	}

	srcFeaturePair = newPairList;
}


struct FrameKeyPointFeatures
{
	vector<cv::KeyPoint> keypointList;
	cv::Mat fFeatures;

	FrameKeyPointFeatures(
		const BaseFrame &frame,
		cv::Ptr<cv::FeatureDetector> fdetector,
		cv::Mat mask)
	{
		frame.computeFeatures(fdetector, keypointList, fFeatures, mask);
	}

	const cv::KeyPoint& keypoint(const int i) const
	{ return keypointList.at(i); }

	Vector2d keypointv(const int i) const
	{ return Vector2d(keypointList[i].pt.x, keypointList[i].pt.y); }
};


void
Matcher::decomposeE (
	const Eigen::Matrix3d &E,
	Eigen::Matrix3d &R1, Eigen::Matrix3d &R2,
	Eigen::Vector3d &t)
{
	JacobiSVD <Matrix3d> svd(E, ComputeFullU|ComputeFullV);
	Matrix3d U, W, V;
	U = svd.matrixU();
	V = svd.matrixV();

	t = U.col(2);
	t /= t.norm();

	W = Matrix3d::Zero();
	W(0,1) = -1;
	W(1,0) = 1;
	W(2,2) = 1;

	R1 = U * W * V.transpose();
	if (R1.determinant() < 0)
		R1 = -R1;
	R2 = U * W.transpose() * V.transpose();
	if (R2.determinant() < 0)
		R2 = -R2;
}


/*
 * Create an epipolar line in Frame 2 based on Fundamental Matrix F12, using a keypoint from Frame 1
 */
Line2 createEpipolarLine (const Matrix3d &F12, const Vector2d &kp1)
{
	Line2 epl2;
	epl2.coeffs() = F12 * kp1.homogeneous();
	epl2.normalize();
	return epl2;
}


Line2 createEpipolarLine (const Matrix3d &F12, const cv::KeyPoint &kp1)
{
	return createEpipolarLine( F12, Vector2d(kp1.pt.x, kp1.pt.y) );
}


double __d;


bool Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2)
{
	return isKeypointInEpipolarLine(epl2, Vector2d(cvkp2.pt.x, cvkp2.pt.y));
}


bool
Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const Eigen::Vector2d &kp2)
{
	auto cof = epl2.coeffs();
	auto d = abs( cof.dot(kp2.homogeneous()) / sqrt(cof[0]*cof[0] + cof[1]*cof[1]) );
//	auto lim = 3.84*VMap::mScaleFactors[cvkp2.octave];
	auto lim = 3.84 * circleOfConfusionDiameter;

	// XXX: Using scale factor makes us more dependent to ORB
	if (d > lim)
		return false;
	else {
		__d = d;
		return true;
	}
}


int
Matcher::matchBruteForce(
	const BaseFrame &F1,
	const BaseFrame &F2,
	Matcher::PairList &featurePairs,
	int windowSize,
	bool checkOrientation
)
{
	if (windowSize==-1) {
		// Take 16% of width
		windowSize = round(0.16 * (float)F1.getCameraParameters().width);
	}

	int nmatches = 0;
	featurePairs.reserve(F1.numOfKeyPoints());

	vector<int> rotationHistogram[Matcher::HISTOGRAM_LENGTH];
	vector<int> matchedDistances(F2.numOfKeyPoints(), numeric_limits<int>::max());
	vector<int> vMatches1(F1.numOfKeyPoints(), -1);
	vector<int> vMatches2(F2.numOfKeyPoints(), -1);

	for (int i=0; i<Matcher::HISTOGRAM_LENGTH; ++i)
		rotationHistogram[i].reserve(500);
	const float factor = 1.0/Matcher::HISTOGRAM_LENGTH;

	for (kpid idx1=0; idx1<F1.numOfKeyPoints(); ++idx1) {
		auto keypoint1 = F1.keypoint(idx1);
		int level = keypoint1.octave;
		if (level > 0)
			continue;

		auto vIndices2 = F2.getKeyPointsInArea(keypoint1.pt.x, keypoint1.pt.y, windowSize, level, level);
		if (vIndices2.empty())
			continue;

		cv::Mat descriptor1 = F1.descriptor(idx1);

		int bestDist = numeric_limits<int>::max(),
			bestDist2 = numeric_limits<int>::max(),
			bestIdx2 = -1;

		for (auto &idx2: vIndices2) {
			cv::Mat descriptor2 = F2.descriptor(idx2);
			int distance = ORBDescriptorDistance(descriptor1, descriptor2);
			if (matchedDistances[idx2]<=distance)
				continue;
			if (distance < bestDist) {
				bestDist2 = bestDist;
				bestDist = distance;
				bestIdx2 = idx2;
			}
			else if (distance < bestDist2) {
				bestDist2 = distance;
			}
		}

		if (bestDist<=Matcher::ORB_DISTANCE_LOW) {
			if (bestDist < float(bestDist2) * 0.75) {
				if (vMatches2[bestIdx2]>=0) {
					vMatches1[vMatches2[bestIdx2]]=-1;
				}
				vMatches1[idx1]=bestIdx2;
				vMatches2[bestIdx2] = idx1;
			}

			if (checkOrientation) {
				float rot = F1.keypoint(idx1).angle - F2.keypoint(bestIdx2).angle;
				if (rot<0.0) rot+=360.0f;
				int bin = round(rot*factor);
				if (bin==Matcher::HISTOGRAM_LENGTH)
					bin = 0;
				assert(bin>=0 and bin<Matcher::HISTOGRAM_LENGTH);
				rotationHistogram[bin].push_back(idx1);
			}
		}
	}

	if (checkOrientation) {
		int ind1=-1, ind2=-1, ind3=-1;

		ComputeThreeMaxima(rotationHistogram, Matcher::HISTOGRAM_LENGTH, ind1, ind2, ind3);
		for (int i=0; i<Matcher::HISTOGRAM_LENGTH; ++i) {
			if (i==ind1 or i==ind2 or i==ind3)
				continue;
			for (uint j=0; j<rotationHistogram[i].size(); ++j) {
				int idx1 = rotationHistogram[i][j];
				if (vMatches1[idx1]>=0) {
					vMatches1[idx1]=-1;
				}
			}
		}
	}

	for (uint idx1=0; idx1<F1.numOfKeyPoints(); idx1++) {
		if (vMatches1[idx1]>=0)
			featurePairs.push_back(make_pair((kpid)idx1, (kpid)vMatches1[idx1]));
	}

	return featurePairs.size();
}


bool isMoving(const vector<pair<cv::Point2f,cv::Point2f>> &flows)
{
	uint c=0;
	for (auto &pr: flows) {
		float dist = cv::norm(pr.first-pr.second);
		if (dist<=1.0)
			c+=1;
	}

	float confidence=float(c)/float(flows.size());
	if (confidence < 0.5) {
//		cout << "Moving: " << confidence << endl;
		return true;
	}
	else {
//		cout << "Not moving: " << confidence << endl;
		return false;
	}
}


cv::Mat
Matcher::createMask(const BaseFrame &F1, const BaseFrame &F2, const PairList &prevMatch)
{
	cv::Mat mmask=cv::Mat::ones(F2.numOfKeyPoints(), F1.numOfKeyPoints(), CV_8UC1);
	if (prevMatch.empty())
		return mmask;

	else {
		mmask = ~mmask;
		for (auto &pmatch: prevMatch) {
			mmask.col(pmatch.second).setTo(0xff);
		}
		return mmask;
	}
}


/*
 * Match features using Sparse Lucas-Kanade Optical Flow
 */
int
Matcher::matchOpticalFlow(
	const BaseFrame &F1,			// F1: train
	const BaseFrame &F2,			// F2: query
	PairList &featurePairs,
	bool *moveDec)
{
	vector<cv::DMatch> bfResult1;
	auto bfMatcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
	bfMatcher->match(F2.allDescriptors(), F1.allDescriptors(), bfResult1);

	vector<uchar> statusOf(F1.numOfKeyPoints());
	vector<float> errOf(F1.numOfKeyPoints());

	// Store targets
	cv::Mat vKeypoints2, p1;
	cv::Mat vKeypoints1 = F1.allKeypointsAsMat();

	cv::calcOpticalFlowPyrLK(F1.getImage(), F2.getImage(),
		vKeypoints1, vKeypoints2,
		statusOf, errOf,
		optFlowWindowSize, maxLevel,
		optFlowStopCriteria);
	cv::calcOpticalFlowPyrLK(F2.getImage(), F1.getImage(),
		vKeypoints2, p1,
		statusOf, errOf,
		optFlowWindowSize, maxLevel,
		optFlowStopCriteria);
	featurePairs.clear();

	assert(vKeypoints1.size()==vKeypoints2.size());
	cv::Mat absDiff(cv::abs(vKeypoints1-p1));

	vector<pair<cv::Point2f,cv::Point2f>> flowChecks;

	for (auto &curMatch: bfResult1) {

		uint keyIdx1 = curMatch.trainIdx,
			keyIdx2 = curMatch.queryIdx;

		if (statusOf[keyIdx1]!=1)
			continue;

		cv::Point2f pointCheck(vKeypoints2.row(keyIdx1));
		if (pointCheck.x<0 or pointCheck.x>=F2.width() or pointCheck.y<0 or pointCheck.y>=F2.height())
			continue;
		if (absDiff.at<float>(keyIdx1,0)>=1 or absDiff.at<float>(keyIdx1,1)>=1)
			continue;

		auto pointTarget = F2.keypoint(keyIdx2).pt;
		if (cv::norm(pointTarget-pointCheck)>4.0)
			continue;

		featurePairs.push_back(make_pair(curMatch.trainIdx, curMatch.queryIdx));
		flowChecks.push_back(make_pair(F1.keypoint(keyIdx1).pt, F2.keypoint(keyIdx2).pt));
	}

	if (moveDec!=nullptr)
		*moveDec = isMoving(flowChecks);

	return featurePairs.size();
}


/*
 * This function matches features from two images using
 * epipolar geometry
 */
void
Matcher::matchEpipolar(
	const BaseFrame &Fr1,
	const BaseFrame &Fr2,
	Matcher::PairList &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher)
{
	featurePairs.clear();

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, initialMatches);
	if (initialMatches.size() < 8) {
		featurePairs = Matcher::PairList();
		return;
	}

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());

	const int MaxBestMatch = initialMatches.size();

	// Select N best matches
	vector<cv::Point2f> pointsIn1(MaxBestMatch), pointsIn2(MaxBestMatch);
	for (int i=0; i<MaxBestMatch; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = Fr1.fKeypoints[m.queryIdx].pt;
		pointsIn2[i] = Fr2.fKeypoints[m.trainIdx].pt;
	}
	cv::Mat mask;
	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter, 0.99, mask);
	// Need Eigen Matrix of F

/*
	for (int i=0; i<initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		if (mask.at<char>(i,0)!=0)
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
	}
	return;
*/

	Matrix3d F12;
	cv2eigen(Fcv, F12);

	/*
	 * Select point pairs that obey epipolar geometry
	 */
	for (int i = 0; i < initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		const Vector2d keypoint1v = Fr1.keypointv(m.queryIdx);
		const Line2 epl2 = createEpipolarLine(F12, Fr1.fKeypoints[m.queryIdx]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(),
				Fr2.fKeypoints[m.trainIdx]);
		if (isKeypointInEpipolarLine(epl2, Fr2.keypointv(m.trainIdx)) == true
				and isKeypointInEpipolarLine(epl1, Fr1.keypointv(m.queryIdx))
					== true) {
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
		}
	}
}


/*
 * Calculate Essential Matrix from F1 & F2, using featurePairs selected by matchAny().
 * We assume that those pairs obey epipolar geometry principle.
 * At the end, inliers are put into the fourth parameter
 */
TTransform
Matcher::calculateMovement (
	const BaseFrame &F1, const BaseFrame &F2,
	const PairList &featurePairs,
	PairList &validPairsByTriangulation)
{
	validPairsByTriangulation = featurePairs;

	vector<cv::Point2f> pointsIn1(featurePairs.size()), pointsIn2(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		auto &m = featurePairs[i];
		pointsIn1[i] = F1.fKeypoints[m.first].pt;
		pointsIn2[i] = F2.fKeypoints[m.second].pt;
	}

	cv::Mat E12, R12e, te, mask;
	E12 = cv::findEssentialMat(pointsIn1, pointsIn2, F1.cameraParam.toCvMat(), cv::RANSAC, 0.9, 4.0, mask);

	int inliers;
	inliers = cv::recoverPose(E12, pointsIn1, pointsIn2, F1.cameraParam.toCvMat(), R12e, te, mask);

	filterFeaturePairMask(validPairsByTriangulation, mask);

	Matrix3d R12;
	Vector3d t;
	cv2eigen(R12e, R12);
	cv2eigen(te, t);

	return TTransform::from_R_t(t, R12);
}


/*
 * Backport from OpenCV 4
 */
int MRecoverPose( cv::InputArray E, cv::InputArray _points1, cv::InputArray _points2,
                            cv::InputArray _cameraMatrix, cv::OutputArray _R, cv::OutputArray _t, double distanceThresh,
                     cv::InputOutputArray _mask, cv::OutputArray triangulatedPoints)
{
	cv::Mat points1, points2, cameraMatrix;
	_points1.getMat().convertTo(points1, CV_64F);
	_points2.getMat().convertTo(points2, CV_64F);
	_cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

	int npoints = points1.checkVector(2);
	CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
			points1.type() == points2.type());

	CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

	if (points1.channels() > 1)
	{
		points1 = points1.reshape(1, npoints);
		points2 = points2.reshape(1, npoints);
	}

	double fx = cameraMatrix.at<double>(0,0);
	double fy = cameraMatrix.at<double>(1,1);
	double cx = cameraMatrix.at<double>(0,2);
	double cy = cameraMatrix.at<double>(1,2);

	points1.col(0) = (points1.col(0) - cx) / fx;
	points2.col(0) = (points2.col(0) - cx) / fx;
	points1.col(1) = (points1.col(1) - cy) / fy;
	points2.col(1) = (points2.col(1) - cy) / fy;

	points1 = points1.t();
	points2 = points2.t();

	cv::Mat R1, R2, t;
	cv::decomposeEssentialMat(E, R1, R2, t);
	cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
	cv::Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
	P1(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
	P2(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
	P3(cv::Range::all(), cv::Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
	P4(cv::Range::all(), cv::Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

	// Do the cheirality check.
	// Notice here a threshold dist is used to filter
	// out far away points (i.e. infinite points) since
	// their depth may vary between positive and negative.
	std::vector<cv::Mat> allTriangulations(4);
	cv::Mat Q;

	cv::triangulatePoints(P0, P1, points1, points2, Q);
	if(triangulatedPoints.needed())
		Q.copyTo(allTriangulations[0]);
	cv::Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask1 = (Q.row(2) < distanceThresh) & mask1;
	Q = P1 * Q;
	mask1 = (Q.row(2) > 0) & mask1;
	mask1 = (Q.row(2) < distanceThresh) & mask1;

	cv::triangulatePoints(P0, P2, points1, points2, Q);
	if(triangulatedPoints.needed())
		Q.copyTo(allTriangulations[1]);
	cv::Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask2 = (Q.row(2) < distanceThresh) & mask2;
	Q = P2 * Q;
	mask2 = (Q.row(2) > 0) & mask2;
	mask2 = (Q.row(2) < distanceThresh) & mask2;

	cv::triangulatePoints(P0, P3, points1, points2, Q);
	if(triangulatedPoints.needed())
		Q.copyTo(allTriangulations[2]);
	cv::Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask3 = (Q.row(2) < distanceThresh) & mask3;
	Q = P3 * Q;
	mask3 = (Q.row(2) > 0) & mask3;
	mask3 = (Q.row(2) < distanceThresh) & mask3;

	cv::triangulatePoints(P0, P4, points1, points2, Q);
	if(triangulatedPoints.needed())
		Q.copyTo(allTriangulations[3]);
	cv::Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
	Q.row(0) /= Q.row(3);
	Q.row(1) /= Q.row(3);
	Q.row(2) /= Q.row(3);
	Q.row(3) /= Q.row(3);
	mask4 = (Q.row(2) < distanceThresh) & mask4;
	Q = P4 * Q;
	mask4 = (Q.row(2) > 0) & mask4;
	mask4 = (Q.row(2) < distanceThresh) & mask4;

	mask1 = mask1.t();
	mask2 = mask2.t();
	mask3 = mask3.t();
	mask4 = mask4.t();

	// If _mask is given, then use it to filter outliers.
	if (!_mask.empty())
	{
		cv::Mat mask = _mask.getMat();
		CV_Assert(npoints == mask.checkVector(1));
		mask = mask.reshape(1, npoints);
		cv::bitwise_and(mask, mask1, mask1);
		cv::bitwise_and(mask, mask2, mask2);
		cv::bitwise_and(mask, mask3, mask3);
		cv::bitwise_and(mask, mask4, mask4);
	}
	if (_mask.empty() && _mask.needed())
	{
		_mask.create(mask1.size(), CV_8U);
	}

	CV_Assert(_R.needed() && _t.needed());
	_R.create(3, 3, R1.type());
	_t.create(3, 1, t.type());

	int good1 = cv::countNonZero(mask1);
	int good2 = cv::countNonZero(mask2);
	int good3 = cv::countNonZero(mask3);
	int good4 = cv::countNonZero(mask4);

	if (good1 >= good2 && good1 >= good3 && good1 >= good4)
	{
		if(triangulatedPoints.needed()) allTriangulations[0].copyTo(triangulatedPoints);
		R1.copyTo(_R);
		t.copyTo(_t);
		if (_mask.needed()) mask1.copyTo(_mask);
		return good1;
	}
	else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
	{
		if(triangulatedPoints.needed()) allTriangulations[1].copyTo(triangulatedPoints);
		R2.copyTo(_R);
		t.copyTo(_t);
		if (_mask.needed()) mask2.copyTo(_mask);
		return good2;
	}
	else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
	{
		if(triangulatedPoints.needed()) allTriangulations[2].copyTo(triangulatedPoints);
		t = -t;
		R1.copyTo(_R);
		t.copyTo(_t);
		if (_mask.needed()) mask3.copyTo(_mask);
		return good3;
	}
	else
	{
		if(triangulatedPoints.needed()) allTriangulations[3].copyTo(triangulatedPoints);
		t = -t;
		R2.copyTo(_R);
		t.copyTo(_t);
		if (_mask.needed()) mask4.copyTo(_mask);
		return good4;
	}
}


/*
 * Perform motion estimation with hint from metric odometry.
 * Movement is calculated as if F1 is identity.
 */
void
Matcher::calculateMovement2(const BaseFrame &F1, const BaseFrame &F2,
	const PairList &featurePairs,
	PairList &validPairsByTriangulation, TTransform &movement, std::vector<Eigen::Vector3d> &points3)
{
//	validPairsByTriangulation = featurePairs;

	vector<cv::Point2f> pointsIn1(featurePairs.size()), pointsIn2(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		auto &m = featurePairs[i];
		pointsIn1[i] = F1.fKeypoints[m.first].pt;
		pointsIn2[i] = F2.fKeypoints[m.second].pt;
	}

	cv::Mat E12, R12e, te, mask, triangulatedPts;
	E12 = cv::findEssentialMat(pointsIn1, pointsIn2, F1.cameraParam.toCvMat(), cv::RANSAC, 0.9, 4.0, mask);

	int inliers;
	double distanceThreshold = 100.0;
	inliers = MRecoverPose(
		E12,
		pointsIn1, pointsIn2,
		F1.cameraParam.toCvMat(),
		R12e, te,
		distanceThreshold,
		mask,
		triangulatedPts);

	int ctr=0;
	points3.clear();
	validPairsByTriangulation.clear();

	for (int i=0; i<mask.rows; ++i) {
		auto m=mask.at<uchar>(i,0);
		if (m!=0) {
			ctr+=1;

			triangulatedPts.col(i) /= triangulatedPts.at<double>(3,i);
			Vector3d pt3(triangulatedPts.at<double>(0,i),
					triangulatedPts.at<double>(1,i),
					triangulatedPts.at<double>(2,i));

			points3.push_back(pt3);
			validPairsByTriangulation.push_back(featurePairs[i]);
		}
	}

	assert(ctr==inliers);

//	filterFeaturePairMask(validPairsByTriangulation, mask);

	Matrix3d R12;
	Vector3d t;
	cv2eigen(R12e, R12);
	cv2eigen(te, t);

	movement = TTransform::from_R_t(t, R12);
	movement = movement.inverse();
}



void
Matcher::matchMapPoints(
	const KeyFrame &KFsrc,
	const BaseFrame &Ft,
	std::vector<KpPair> &featurePairs)
{
	featurePairs.clear();

	Matcher::PairList initialPairs, checkedMpPairs;
	int N = Matcher::matchBruteForce(KFsrc, Ft, initialPairs);
	for (auto &pair: initialPairs) {
		try {
			mpid mp = KFsrc.parent()->getMapPointByKeypoint(KFsrc.getId(), pair.first);
			checkedMpPairs.push_back(pair);
		} catch (...) { continue; }
	}

	N = checkedMpPairs.size();
	if (N<10) return;
	vector<cv::Point2f> pointsIn1(N), pointsIn2(N);
	for (int i=0; i<checkedMpPairs.size(); i++) {
		auto &pr = checkedMpPairs[i];
		pointsIn1[i] = KFsrc.fKeypoints[pr.first].pt;
		pointsIn2[i] = Ft.fKeypoints[pr.second].pt;
	}

	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter);
	if (Fcv.empty()) return;
	Matrix3d F12;
	cv2eigen(Fcv, F12);

	// Check matching against epipolar constraints
	float maxDistance = -1;
	for (int i=0; i<N; ++i) {
		auto &pr = checkedMpPairs[i];
		const Vector2d keypoint1v = KFsrc.keypointv(pr.first);
		const Line2 epl2 = createEpipolarLine(F12, KFsrc.fKeypoints[pr.first]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(), Ft.fKeypoints[pr.second]);
		if (isKeypointInEpipolarLine(epl2, Ft.keypointv(pr.second))==true and isKeypointInEpipolarLine(epl1, KFsrc.keypointv(pr.first))==true) {
			featurePairs.push_back(pr);
		}
	}
}


void
Matcher::solvePose(
	const KeyFrame &KFsrc,
	const BaseFrame &Ft,
	std::vector<KpPair> &featurePairs,
	Pose &newFramePose)
{
	Eigen::Matrix4d eKfExt = KFsrc.externalParamMatrix4();
	Matrix3d KfRotation = eKfExt.block<3,3>(0,0);
	cv::Mat cKfRotMat, cRVec;
	cv::eigen2cv(KfRotation, cKfRotMat);
	cv::Rodrigues(cKfRotMat, cRVec);

	Eigen::Vector3d eKfTransVec = eKfExt.block<3,1>(0,3);
	cv::Mat cKfTransVec;
	cv::eigen2cv(eKfTransVec, cKfTransVec);

	cv::Mat
		objectPoints (featurePairs.size(), 3, CV_32F),
		imagePoints (featurePairs.size(), 2, CV_32F);
	for (int r=0; r<featurePairs.size(); ++r) {
		mpid mp = KFsrc.parent()->getMapPointByKeypoint(KFsrc.getId(), featurePairs[r].first);
		auto P = KFsrc.parent()->mappoint(mp);
		objectPoints.at<float>(r, 0) = P->X();
		objectPoints.at<float>(r, 1) = P->Y();
		objectPoints.at<float>(r, 2) = P->Z();
		imagePoints.at<float>(r, 0) = Ft.keypoint(featurePairs[r].second).pt.x;
		imagePoints.at<float>(r, 1) = Ft.keypoint(featurePairs[r].second).pt.y;
	}

	cv::Mat cameraMatrix = KFsrc.getCameraParameters().toCvMat();

	cv::Mat inlierIdx;

	bool hasSolution = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::Mat(), cRVec, cKfTransVec, true, 100, 4.0, 0.99, inlierIdx, cv::SOLVEPNP_EPNP);
	if (hasSolution==false)
		throw runtime_error("No solutions found");
	auto t2=getCurrentTime();

	cv::Rodrigues(cRVec, cKfRotMat);
	cv::cv2eigen(cKfRotMat, KfRotation);
	Eigen::Quaterniond Q;
	Q = KfRotation;
	cv::cv2eigen(cKfTransVec, eKfTransVec);

	// XXX: Inverse or not?
	newFramePose = Pose::from_Pos_Quat(eKfTransVec, Q).inverse();

	vector<KpPair> inliers(inlierIdx.rows);
	for (int r=0; r<inlierIdx.rows; ++r) {
		inliers[r] = featurePairs.at(inlierIdx.at<int>(r,0));
	}

	featurePairs = inliers;
}


void
Matcher::rotationFinder
(const BaseFrame &Fr1, const BaseFrame &Fr2,
const std::vector<KpPair> &featurePairs,
double &theta, double &phi)
{
	// estimate phi using normal
	phi = acos(Fr1.normal().dot(Fr2.normal()));
	theta = phi*2;

	const int numPairs = min(500, static_cast<int>(featurePairs.size()));

	MatrixX3d PF1, PF2;
	PF1.resize(numPairs, Eigen::NoChange);
	PF2.resize(numPairs, Eigen::NoChange);
	for (int i=0; i<numPairs; ++i) {
		Vector3d P1 = Fr1.keypointn(featurePairs[i].first);
		Vector3d P2 = Fr2.keypointn(featurePairs[i].second);
		PF1.row(i) = P1;
		PF2.row(i) = P2;
	}

	VectorXd F;
	F.resize(numPairs);
	MatrixX2d J;
	J.resize(numPairs, Eigen::NoChange);

	for (int i=0; i<10; ++i) {
		// the function
		for (int j=0; j<numPairs; ++j) {
			Vector3d P1 = PF1.row(j), P2 = PF2.row(j);
			F[j] = -P1.x()*P2.y()* cos(phi) + P1.y()*P2.x()*cos(theta-phi) + P1.z()*P2.y()*sin(phi) + P1.y()*P2.z()*sin(theta-phi);

			J(j, 0) = -P1.y()*P2.x()*sin(theta-phi) + P1.y()*P2.z()*cos(theta-phi);
			J(j, 1) = P1.x()*P2.y()*sin(phi) + P1.y()*P2.x()*sin(theta-phi) - P1.y()*P2.z()*cos(theta-phi) + P1.z()*P2.y()*cos(phi);
		}

		MatrixXd Jinv = pseudoInverse(J);
		Vector2d X = Jinv * F;
		theta = theta - X[0];
		phi = phi - X[1];
	}

	return;
}


double
Matcher::getCameraBaselinkOffset
(const Pose &baselinkPose1, const Pose &baselinkPose2, const double &theta, const double &phi)
{
	double
		rho = (baselinkPose2.position()-baselinkPose1.position()).norm();
	double
		L = rho * (-sin(theta/2 - phi) / ( sin(phi) + sin(theta-phi) ));

	// XXX: Get L from somewhere else
	double
		lambda = -2 * L * sin(theta/2) / sin(theta/2 - phi);

	return L;
}



cv::Mat
Matcher::drawMatches(
	const BaseFrame &F1,
	const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	DrawMode mode,
	int maxNumOfPairs)
{
	cv::Mat result(std::max(F1.height(), F2.height()), F1.width()+F2.width(), F1.image.type());
	F1.image.copyTo( result(cv::Rect(0,0,F1.width(),F1.height())) );
	F2.image.copyTo( result(cv::Rect(F1.width(),0,F2.width(),F2.height())) );
//	cv::Mat result = F2.image.clone();
	if (maxNumOfPairs<0)
		maxNumOfPairs = featurePairs.size();
	else
		maxNumOfPairs = min(maxNumOfPairs, static_cast<int>(featurePairs.size()));

	vector<pair<cv::Point2f, cv::Point2f>> pointPairList(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		cv::Point2f p2 = F2.fKeypoints[featurePairs[i].second].pt;
		p2.x += F1.width();
		pointPairList[i] = make_pair(
			F1.fKeypoints[featurePairs[i].first].pt,
			p2);
	}

	// Draw P2 in P1
	Pose P1z = F2.pose();
	Vector2d C2in1 = F1.project(P1z.position());
	cv::circle(result, cv::Point2f(C2in1.x(), C2in1.y()), pointRadius*2, colorYellow);

	if (mode==DrawOpticalFlow) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::Point2f pt1s = pr.first;
			pt1s.x += F1.width();
			cv::line(result, pt1s, pr.second, colorGreen);
		}
	}

	else if (mode==DrawSideBySide) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::line(result, pr.first, pr.second, colorGreen);
		}
	}

	else if (mode==DrawOnlyPoints) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
		}
	}

	else throw runtime_error("Invalid mode");

	return result;
}


int
Matcher::CheckRT (
	const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	std::vector<bool> &goodFeaturePairs,
	float &parallax)
{
	int nGood = 0;
	Vector3d
		origin1 = Vector3d::Zero(),
		origin2 = -R.transpose() * t;

	// Build new projection matrices
	// For camera 1: K[I | O]
	BaseFrame::ProjectionMat P1 = BaseFrame::ProjectionMat::Zero();
	P1.block<3,3>(0,0) = F1.cameraParam.toMatrix3();

	// For camera 2: K[R | t]
	BaseFrame::ProjectionMat P2 = BaseFrame::ProjectionMat::Zero();
	P2.block<3,3>(0,0) = R;
	P2.block<3,1>(0,3) = t;
	P2 = F2.cameraParam.toMatrix3() * P2;

	vector<double> vCosParallax(featurePairs.size());

	/*
	 * Triangulate each point pair as if first camera is in the origin
	 */
	for (int ip=0; ip<featurePairs.size(); ++ip) {

		Vector2d
			pt1 = F1.keypointv(featurePairs[ip].first),
			pt2 = F2.keypointv(featurePairs[ip].second);

		Vector4d point3D_;
		TriangulateDLT(P1, P2, pt1, pt2, point3D_);
		Vector3d point3D = (point3D_ / point3D_[3]).hnormalized();

		if (!isfinite(point3D[0]) or !isfinite(point3D[1]) or !isfinite(point3D[2]) )
			continue;

		Vector3d
			normal1 = point3D - origin1,
			normal2 = point3D - origin2;
		double
			dist1 = normal1.norm(),
			dist2 = normal2.norm();

		double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

		// Check that depth regarding camera1&2 must be positive
		if (cosParallax < 0.99998) {
			if (point3D[2]<=0)
				continue;
			Vector3d point3D2 = R*point3D + t;
			if (point3D2[2]<=0)
				continue;
		}

		// Check reprojection errors
		Vector3d
			proj1 = P1 * point3D.homogeneous(),
			proj2 = P2 * point3D.homogeneous();
		proj1 /= proj1[2];
		proj2 /= proj2[2];

		float
			squareError1 = pow((proj1.hnormalized()-pt1).norm(), 2),
			squareError2 = pow((proj2.hnormalized()-pt2).norm(), 2),
			threshold = pow(Matcher::circleOfConfusionDiameter, 2);
		if (squareError1 > threshold or squareError2 > threshold)
			continue;

		nGood++;
		goodFeaturePairs[ip] = true;
	}

	if (nGood > 0) {
		// XXX: Unfinished, we do not collect parallaxes
	}
	else
		parallax = 0;

	return nGood;
}


// Match with homography constraints
void
Matcher::matchH(
	const BaseFrame &Fs1, const BaseFrame &Fs2,
	cv::Mat planeMask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	cv::Ptr<cv::DescriptorMatcher> fmatcher,
	Eigen::Matrix3d &H)
{
	FrameKeyPointFeatures
		F1(Fs1, fdetector, planeMask),
		F2(Fs2, fdetector, planeMask);

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	fmatcher->match(F1.fFeatures, F2.fFeatures, initialMatches);

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());

	const int MaxBestMatch = 500;
	int howmany = std::min(MaxBestMatch, static_cast<int>(initialMatches.size()));

	// Select N best matches
	vector<cv::Point2f> pointsIn1(howmany), pointsIn2(howmany);
	for (int i=0; i<howmany; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = F1.keypoint(m.queryIdx).pt;
		pointsIn2[i] = F2.keypoint(m.trainIdx).pt;
	}
	cv::Mat inlierMask;
	cv::Mat Hcv = cv::findHomography(pointsIn1, pointsIn2, cv::RANSAC, 3, inlierMask);
	cv2eigen(Hcv, H);

	// Check how many inliers we have
	int nGood = 0;
	for (int i=0; i<howmany; ++i) {
		if(inlierMask.at<int>(i,0) != 0)
			nGood++;
	}

	return;
}


/*
TTransform
Matcher::matchLidarScans(const MeidaiDataItem &frame1, const MeidaiDataItem &frame2)
{
	ptime scantime_frame1, scantime_frame2;
	auto
		pcscan1 = const_cast<MeidaiDataItem&>(frame1).getLidarScan(&scantime_frame1),
		pcscan2 = const_cast<MeidaiDataItem&>(frame2).getLidarScan(&scantime_frame2);

	pcscan1 = LidarScanBag::VoxelGridFilter(pcscan1);
	pcscan2 = LidarScanBag::VoxelGridFilter(pcscan2);

//	pcl::NormalDistributionsTransform<LidarScanBag::point3_t, LidarScanBag::point3_t> ndt;
	pcl_omp::NormalDistributionsTransform<LidarScanBag::point3_t, LidarScanBag::point3_t> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);
	ndt.setInputSource(pcscan1);
	ndt.setInputTarget(pcscan2);

	LidarScanBag::scan_t finalCl;
	TTransform guess12;
	ndt.align(finalCl);

	if (ndt.hasConverged()) {
		cout << "Converged; Score: " << ndt.getFitnessScore() << endl;
		guess12 = ndt.getFinalTransformation().cast<double>();
	}

	else {
		cout << "Not converged" << endl;
		return Matrix4d::Identity();
	}

	double velocity = guess12.translation().norm() / toSeconds(scantime_frame2-scantime_frame1);
	guess12.translation() = guess12.translation().normalized() * velocity * toSeconds(frame2.getTimestamp()-frame1.getTimestamp());
	return guess12;
}
*/


void
Matcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


void
Matcher::matchViso(const BaseFrame &F1, const BaseFrame &F2, PairList &featurePairs)
{

}

} /* namespace Vmml */
