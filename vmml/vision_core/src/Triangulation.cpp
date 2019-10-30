/*
 * triangulation.cpp
 *
 *  Created on: Jun 14, 2018
 *      Author: sujiwo
 */


/*
 * Taken from TheiaSfM
 */

#include <string>
#include <exception>
#include "Triangulation.h"
#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace Eigen;


namespace Vmml {


// Given either a fundamental or essential matrix and two corresponding images
// points such that ematrix * point2 produces a line in the first image,
// this method finds corrected image points such that
// corrected_point1^t * ematrix * corrected_point2 = 0.
void FindOptimalImagePoints(const Matrix3d& ematrix,
                            const Vector2d& point1,
                            const Vector2d& point2,
                            Vector2d* corrected_point1,
                            Vector2d* corrected_point2)
{
	const Vector3d point1_homog = point1.homogeneous();
	const Vector3d point2_homog = point2.homogeneous();

	// A helper matrix to isolate certain coordinates.
	Matrix<double, 2, 3> s_matrix;
	s_matrix << 1, 0, 0, 0, 1, 0;

	const Eigen::Matrix2d e_submatrix = ematrix.topLeftCorner<2, 2>();

	// The epipolar line from one image point in the other image.
	Vector2d epipolar_line1 = s_matrix * ematrix * point2_homog;
	Vector2d epipolar_line2 = s_matrix * ematrix.transpose() * point1_homog;

	const double a = epipolar_line1.transpose() * e_submatrix * epipolar_line2;
	const double b =
	  (epipolar_line1.squaredNorm() + epipolar_line2.squaredNorm()) / 2.0;
	const double c = point1_homog.transpose() * ematrix * point2_homog;

	const double d = sqrt(b * b - a * c);

	double lambda = c / (b + d);
	epipolar_line1 -= e_submatrix * lambda * epipolar_line1;
	epipolar_line2 -= e_submatrix.transpose() * lambda * epipolar_line2;

	lambda *=
	  (2.0 * d) / (epipolar_line1.squaredNorm() + epipolar_line2.squaredNorm());

	*corrected_point1 =
	  (point1_homog - s_matrix.transpose() * lambda * epipolar_line1)
		  .hnormalized();
	*corrected_point2 =
	  (point2_homog - s_matrix.transpose() * lambda * epipolar_line2)
		  .hnormalized();
}


Matrix3d CrossProductMatrix(const Vector3d& cross_vec)
{
	Matrix3d cross;
	cross << 0.0, -cross_vec.z(), cross_vec.y(),
		cross_vec.z(), 0.0, -cross_vec.x(),
		-cross_vec.y(), cross_vec.x(), 0.0;
	return cross;
}


void EssentialMatrixFromTwoProjectionMatrices(
	const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	Matrix3d* essential_matrix)
{
	// Create the Ematrix from the poses.
	const Matrix3d R1 = pose1.leftCols<3>();
	const Matrix3d R2 = pose2.leftCols<3>();
	const Vector3d t1 = pose1.rightCols<1>();
	const Vector3d t2 = pose2.rightCols<1>();

	// Relative transformation between to cameras.
	const Matrix3d relative_rotation = R1 * R2.transpose();
	const Vector3d translation = (t1 - relative_rotation * t2).normalized();
	*essential_matrix = CrossProductMatrix(translation) * relative_rotation;
}


bool Triangulate(const Matrix3x4d& pose1,
                 const Matrix3x4d& pose2,
                 const Vector2d& point1,
                 const Vector2d& point2,
                 Vector4d& triangulated_point)
{
	Matrix3d ematrix;
	EssentialMatrixFromTwoProjectionMatrices(pose1, pose2, &ematrix);

	Vector2d corrected_point1, corrected_point2;
	FindOptimalImagePoints (ematrix, point1, point2, &corrected_point1, &corrected_point2);

	// Now the two points are guaranteed to intersect. We can use the DLT method
	// since it is easy to construct.
	return TriangulateDLT(pose1, pose2, corrected_point1, corrected_point2, triangulated_point);
}


bool TriangulateDLT(const Matrix3x4d& projMat1,
                    const Matrix3x4d& projMat2,
                    const Vector2d& point1,
                    const Vector2d& point2,
                    Vector4d& triangulated_point)
{
	Matrix4d design_matrix;
	design_matrix.row(0) = point1[0] * projMat1.row(2) - projMat1.row(0);
	design_matrix.row(1) = point1[1] * projMat1.row(2) - projMat1.row(1);
	design_matrix.row(2) = point2[0] * projMat2.row(2) - projMat2.row(0);
	design_matrix.row(3) = point2[1] * projMat2.row(2) - projMat2.row(1);

	// Extract nullspace
	JacobiSVD<Matrix4d,false> svd(design_matrix, Eigen::ComputeFullV);
	triangulated_point = svd.matrixV().col(3);
	triangulated_point = triangulated_point / (triangulated_point[3]);
	return true;
}


bool TriangulateCV(
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<Matcher::KpPair> &featurePairs,
	std::map<uint, Eigen::Vector3d> &trianglPoints,
	float *parallax)
{
	const int N = featurePairs.size();
	const auto
		pm1 = F1.projectionMatrix(),
		pm2 = F2.projectionMatrix();
	cv::Mat projMat1, projMat2;
	vector<cv::Point2f> pointsInFrame1(N), pointsInFrame2(N);
	cv::Mat triangulationResults;

	float cosParallaxRet = 0.0;

	cv::eigen2cv(pm1, projMat1);
	cv::eigen2cv(pm2, projMat2);

	int i=0;
	for (auto &pointPair: featurePairs) {
		pointsInFrame1[i] = F1.keypoint(pointPair.first).pt;
		pointsInFrame2[i] = F2.keypoint(pointPair.second).pt;
		++i;
	}

	cv::triangulatePoints(projMat1, projMat2, pointsInFrame1, pointsInFrame2, triangulationResults);
	if (N!=triangulationResults.cols)
		throw runtime_error("Unexpected number of points; should be "+to_string(featurePairs.size()));

	/*
	 * Check triangulation results
	 */
	for (i=0; i<N; ++i) {
		cv::Vec4f p = triangulationResults.col(i);
		p /= p[3];
		Vector3d pointm(p[0], p[1], p[2]);

		// Purge invalid numbers
		if (!isfinite(pointm.x()) or !isfinite(pointm.y()) or !isfinite(pointm.z()))
			continue;

		auto
			proj1 = F1.keypointv(featurePairs[i].first),
			proj2 = F2.keypointv(featurePairs[i].second);
		float
			e1 = float(F1.keypoint(featurePairs[i].first).octave+1) * Matcher::circleOfConfusionDiameter,
			e2 = float(F2.keypoint(featurePairs[i].second).octave+1) * Matcher::circleOfConfusionDiameter;

		// Check for Reprojection Errors
		float pj1 = (F1.project(pointm) - proj1).norm(),
			pj2 = (F2.project(pointm) - proj2).norm();
		if (pj1 > e1 or pj2 > e2)
			continue;

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - F1.position();
		Vector3d trans1 = F1.transform(pointm);
		if (trans1.z() < 0)
			continue;

		Vector3d v2 = pointm - F2.position();
		Vector3d trans2 = F2.transform(pointm);
		if (trans2.z() < 0)
			continue;

		// 2: Must have enough parallax (ie. remove faraway points)
		double cosParallax = (-v1).dot(-v2) / (v1.norm() * v2.norm());
		if (cosParallax >= 0.999990481)
			continue;
		if (cosParallax > cosParallaxRet)
			cosParallaxRet = cosParallax;

		trianglPoints.insert(make_pair(i, pointm));

		// XXX: Find maximum parallax value
/*
		mpid newMp = parent.createMapPoint(pointm);
		newMapPointList.push_back(newMp);
		mapPointToKeyPointInKeyFrame1[newMp] = kpPair[i].first;
		mapPointToKeyPointInKeyFrame2[newMp] = kpPair[i].second;
*/
	}

	if (parallax!=NULL)
		*parallax = acos(cosParallaxRet) * 180.0/M_PI;

	return true;
}


}		// namespace Vmml




