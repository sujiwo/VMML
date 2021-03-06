/*
 * Matcher.h
 *
 *  Created on: Oct 9, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_MATCHER_H_
#define VMML_CORE_MATCHER_H_

#include <vector>
#include <map>
#include <set>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utilities.h"
#include "BaseFrame.h"
#include "KeyFrame.h"


namespace Vmml
{

class Matcher {
public:
	typedef std::pair<kpid,kpid> KpPair;
	typedef std::vector<KpPair> PairList;
	typedef std::map<kpid,kpid> PairListIndexed;

	const static int
		HISTOGRAM_LENGTH	=	30,
		ORB_DISTANCE_HIGH	=	100,
		ORB_DISTANCE_LOW	=	50;

	// Matching features for initialization
	static int
	matchBruteForce(
		const BaseFrame &F1,
		const BaseFrame &F2,
		PairList &featurePairs,
		int windowSize=-1,
		bool checkOrientation=true
	);

	/*
	 * Create mask for matching from F1 to F2.
	 * If prevMatch is empty, resulting mask
	 * is filled by ones.
	 * Otherwise, it is filled by features in F1 that has
	 * non-empty pair in prevMatch (prevMatch must be
	 * filled from match from F0 to F1)
	 */
	static cv::Mat createMask(const BaseFrame &F1, const BaseFrame &F2, const PairList &prevMatch=PairList());

	/*
	 * Match features using Sparse Lucas-Kanade Optical Flow
	 */
	static int
	matchOpticalFlow(
		const BaseFrame &F1,
		const BaseFrame &F2,
		PairList &featurePairs,
		bool *isMoving=nullptr);

	static void matchViso(const BaseFrame &F1, const BaseFrame &F2, PairList &featurePairs);

	// Match with epipolar constraints
	static void
	matchEpipolar(
		const BaseFrame &F1,
		const BaseFrame &F2,
		PairList &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher);

	/*
	 * Perform motion estimation using essential matrix
	 */
	static TTransform
	calculateMovement (
		const BaseFrame &F1, const BaseFrame &F2,
		const PairList &featurePairs,
		PairList &validPairsByTriangulation);

	/*
	 * Perform motion estimation and triangulation
	 */
	static void
	calculateMovement2(const BaseFrame &F1, const BaseFrame &F2,
		const PairList &featurePairs,
		PairList &validPairsByTriangulation,
		TTransform &movement, std::vector<Eigen::Vector3d> &points3);

	// Match with homography constraints.
	// We have to compute new features based on custom mask
	static void
	matchH(
		const BaseFrame &F1, const BaseFrame &F2,
		cv::Mat planeMask,
		cv::Ptr<cv::FeatureDetector> fdetector,
		cv::Ptr<cv::DescriptorMatcher> fmatcher,
		Eigen::Matrix3d &H);

	template<typename PointT>
	static TTransform
	matchLidarScans(const pcl::PointCloud<PointT> &frame1, const pcl::PointCloud<PointT> &frame2);

	static void
	matchMapPoints(
		const KeyFrame &KFsrc,
		const BaseFrame &Ft,
		PairList &featurePairs);

	/*
	 * Solve pose from matched keypoints in KFsrc to Ft
	 * keypoints in KFsrc must be related to a mappoint
	 */
	static void
	solvePose(
		const KeyFrame &KFsrc,
		const BaseFrame &Ft,
		PairList &featurePairs,
		Pose &newFramePose);

	enum DrawMode {
		DrawOpticalFlow,
		DrawSideBySide,
		DrawOnlyPoints,
		DrawEpipolarIn2
	};

	static cv::Mat
	drawMatches(
		const BaseFrame &F1,
		const BaseFrame &F2,
		const PairList &featurePairs,
		DrawMode m,
		int maxNumOfPairs=-1);

	static void
	decomposeE (const Eigen::Matrix3d &E, Eigen::Matrix3d &R1, Eigen::Matrix3d &R2, Eigen::Vector3d &t);

	/*
	 * Calculate sine & cosine 1 & 2 from rotation
	 */
	static void
	rotationFinder (const BaseFrame &F1, const BaseFrame &F2, const std::vector<KpPair> &featurePairs, double &theta, double &phi);

	static double
	getCameraBaselinkOffset (const Pose &baselinkPose1, const Pose &baselinkPose2, const double &theta, const double &phi);

	static float
	circleOfConfusionDiameter;

	static int
	__maxDraw;

protected:

	static bool
	isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2);

	static bool
	isKeypointInEpipolarLine (const Line2 &epl2, const Eigen::Vector2d &kp2);

	static int
	CheckRT (
		const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
		const BaseFrame &F1, const BaseFrame &F2,
		const std::vector<KpPair> &featurePairs,
		std::vector<bool> &goodFeaturePairs,
		float &parallax);

	static void
	ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
};

} /* namespace Vmml */

#endif /* VMML_CORE_MATCHER_H_ */
