/*
 * utilities.h
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#ifndef VMML__CORE_UTILITIES_H_
#define VMML__CORE_UTILITIES_H_

#include <set>
#include <map>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>
#include <memory>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/conversion.hpp>

#include <ros/package.h>

using std::pair;
using std::set;
using std::map;
using std::vector;


namespace Vmml {


typedef uint64_t kfid;
typedef uint64_t mpid;
typedef decltype(cv::DMatch::trainIdx) kpid;
typedef uint64_t sourceId;

typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::time_duration tduration;

const auto
	MIN_TIME = boost::posix_time::special_values::min_date_time,
	MAX_TIME = boost::posix_time::special_values::max_date_time;

const ptime
	unixTime0(boost::gregorian::date(1970,1,1));


template<typename P, typename Q>
map<Q,P> reverseMap (const map<P,Q> &smap)
{
	map<Q,P> nmap;
	for (auto &pq: smap) {
		P p = pq.first;
		Q q = pq.second;
		nmap[q] = p;
	}
	return nmap;
}


template<typename K, typename V>
set<K> allKeys (const map<K,V> &smap)
{
	set<K> retval;
	for (auto &p: smap) {
		retval.insert(p.first);
	}
	return retval;
}


template<typename T>
set<T> toSet (const vector<T> &vect)
{
	set<T> Res;
	for (T &v: vect) {
		Res.insert(v);
	}
	return Res;
}


template<typename T>
bool inSet (const set<T> &S, const T &val)
{
	return (S.find(val) != S.end());
}


// Result = A - B
template<typename T>
set<T> subtract (const set<T> &A, const set<T> &B)
{
	set<T> Res;
	for (auto &p: A) {
		if (!inSet(B, p)) {
			Res.insert(p);
		}
	}
	return Res;
}


template<typename K, typename V>
vector<V> allValues (const map<K,V> &sMap)
{
	vector<V> allV;
	for (auto &kv: sMap) {
		allV.push_back(kv.second);
	}
	return allV;
}


template<typename Scalar>
using VectorXx = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;


/*
 * Median of a Eigen-based vector
 */
template <typename Scalar>
double median(const VectorXx<Scalar> &v)
{
	int n = v.rows();
	vector<Scalar> vs (v.data(), v.data()+v.rows());
	sort(vs.begin(), vs.end());
	if (n%2==1)
		return (double)vs[(n-1)/2];
	else
		return (double(vs[n/2])+double(vs[(n/2)-1])) / 2;
}


template <typename k, typename v>
pair<const k,v>
maximumMapElement(const map<k,v> &maptg)
{
	auto p = max_element(maptg.begin(), maptg.end(),
		[](const pair<k,v> &element1, const pair<k,v> &element2)
			{return element1.second < element2.second;}
	);
	return *p;
}


/*
 * Calculate normalized cumulative distribution function over a histogram of an image
 */
Eigen::VectorXd cdf (const cv::Mat &grayImage, const cv::Mat &mask=cv::Mat());


inline ptime getCurrentTime ()
{ return boost::posix_time::microsec_clock::local_time(); }

inline double td_seconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }

double toSeconds (const ptime &pt);

inline double toSeconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }

/*
 * Convert UNIX timestamp in seconds to ptime
 */
ptime fromSeconds (const double s);

void debugMsg(const std::string &s, double is_error=false);


#define RecordRuntime(funcDef, CALL)  \
	ptime _t1_ = getCurrentTime(); \
	CALL ; \
	ptime _t2_ = getCurrentTime(); \
	tduration _td_ = _t2_ - _t1_ ; \
	debugMsg(string(funcDef) + " (seconds): " + to_string(double(_td_.total_microseconds()) / 1e6)); \


inline boost::filesystem::path getMyPath()
{ return boost::filesystem::path(ros::package::getPath("vmml")); }


/*
 * Our own pseudo-inverse routine, in case Eigen does not provide it
 */
template<typename Scalar, int r, int c>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
pseudoInverse(const Eigen::Matrix<Scalar,r,c> &M, const Scalar cutoff=1e-10)
{
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pinv;

	Eigen::JacobiSVD <Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(M, Eigen::ComputeFullU|Eigen::ComputeFullV);
	auto U = svd.matrixU();
	auto V = svd.matrixV();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Sx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(M.cols(), M.rows());
	for (auto i=0; i<M.cols(); ++i) {
		Scalar s = svd.singularValues()[i];
		if (fabs(s)<=cutoff)
			Sx(i,i) = 0;
		else Sx(i,i) = 1/s;
	}
	pinv = V * Sx * U.transpose();

	return pinv;
}


template<typename T>
class Grid
{
public:
	Grid()
	{}

	Grid(int width, int height, T*(constFunc)(int x, int y)=NULL)
	{
		mGrid.resize(height);
		for (int y=0; y<height; y++) {
			mGrid[y].resize(width);
			for (int x=0; x<width; x++) {
				if (constFunc==NULL) {
					mGrid[y][x].reset(new T);
				}
				else
					mGrid[y][x].reset(constFunc(x, y));
			}
		}
	}

	const T& operator() (const int x, const int y) const
	{ return *mGrid[y][x]; }

	T& operator() (const int x, const int y)
	{ return at(x, y); }

	T& at(const int x, const int y)
	{ return *mGrid[y][x]; }

	int width() const
	{ return mGrid[0].size(); }

	int height() const
	{ return mGrid.size(); }

protected:
	std::vector<std::vector<std::shared_ptr<T>>> mGrid;

};



}		// namespace Vmml

#endif /* VMML__CORE_UTILITIES_H_ */
