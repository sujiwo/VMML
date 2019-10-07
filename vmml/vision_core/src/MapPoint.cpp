/*
 * MapPoint.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#include <limits>

#include "MapPoint.h"


using namespace std;
using namespace Eigen;


namespace Vmml {

mpid MapPoint::nextId = 1;


MapPoint::MapPoint() {
	// TODO Auto-generated constructor stub

}


MapPoint::MapPoint(const Vector3d &p) :
	position(p),
	id(nextId++)
{}

MapPoint::~MapPoint() {}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBDescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}


template <typename Derived>
double median (const Eigen::MatrixBase<Derived> &v)
{
	int n = v.size();

	vector<typename Derived::Scalar> vs(n);

	if (v.Flags & Eigen::MatrixBase<Derived>::IsRowMajor) {
		for (int i=0; i<v.rows(); ++i)
			for (int j=0; j<v.cols(); ++j)
				vs[i*v.cols() + j] = v(i,j);
	}

	else {
		for (int j=0; j<v.cols(); ++j)
			for (int i=0; i<v.rows(); ++i)
				vs[j*v.rows() + i] = v(i,j);
	}

	sort(vs.begin(), vs.end());

	if (n%2==1)
		return (static_cast<double> (vs[(n-1)/2]) );
	else
		return static_cast<double>( (vs[n/2]) + (vs[(n/2)-1]) ) / 2;
}


void
MapPoint::createDescriptor(const std::vector<KeyMapPoint> &visibleIn)
{
	vector<cv::Mat> allDescriptors;
	for (auto &kmp: visibleIn) {
		allDescriptors.push_back(kmp.keyframe.descriptor(kmp.keypointIdx));
	}

	if (allDescriptors.empty())
		return;

	const size_t N = allDescriptors.size();
	MatrixXi MDistances
		= MatrixXi::Zero(N,N);
	for (int i=0; i<N; i++) {
		for (int j=i+1; j<N; j++) {
			MDistances(i,j)
				= MDistances(j,i)
				= ORBDescriptorDistance(allDescriptors[i], allDescriptors[j]);
		}
	}

	double BestMedian = numeric_limits<double>::max();
	int medIdx;
	for (int j=0; j<N; j++) {
		double cMedian = median(MDistances.col(j));
		if (cMedian < BestMedian) {
			BestMedian = cMedian;
			medIdx = j;
		}
	}

	descriptor = allDescriptors[medIdx].clone();
}

} /* namespace Vmml */
