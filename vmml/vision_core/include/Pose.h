/*
 * Pose.h
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

/*
 * All geometry manipulations and classes
 * go in here
 */

#ifndef VMML_CORE_POSE_H_
#define VMML_CORE_POSE_H_

#include "utilities.h"
#include <Eigen/Geometry>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>


using Eigen::Quaterniond;
using Eigen::Vector3d;


namespace Vmml {


// Plane and Line manipulation
typedef Eigen::Hyperplane<double, 3> Plane3;
typedef Eigen::Hyperplane<double, 2> Line2;

// Lines in 3D are represented as parametrized lines
typedef Eigen::ParametrizedLine<double, 3> Line3;

/*
 * All angles are in Radian
 */
Quaterniond fromRPY (double roll, double pitch, double yaw);

Vector3d quaternionToRPY (const Quaterniond &q);

class TQuaternion : public Eigen::Quaterniond
{
public:
	inline TQuaternion(const double &x, const double &y, const double &z, const double &w):
		Quaterniond(w, x, y, z)
	{}

	inline TQuaternion(double roll, double pitch, double yaw)
	{
		Quaterniond q = fromRPY(roll, pitch, yaw);
		coeffs() = q.coeffs();
	}

	inline TQuaternion(const Quaterniond &q)
	{ coeffs() = q.coeffs(); }

	inline TQuaternion(const Eigen::Matrix3d &R)
	{
		Quaterniond Q(R);
		coeffs() = Q.coeffs();
	}

	inline TQuaternion& operator=(const Quaterniond &q)
	{
		coeffs() = q.coeffs();
		return *this;
	}

	inline double roll() const
	{ return quaternionToRPY(*this).x(); }

	inline double pitch() const
	{ return quaternionToRPY(*this).y(); }

	inline double yaw() const
	{ return quaternionToRPY(*this).z(); }

	TQuaternion operator * (const double &mul) const;
	TQuaternion operator / (const double &div) const;

	Eigen::Vector4d asVector() const
	{ return Eigen::Vector4d(x(), y(), z(), w()); }
};


struct TTransform : public Eigen::Affine3d
{
	TTransform()
	{ m_matrix = Eigen::Matrix4d::Identity(); }

	TTransform(const Eigen::Affine3d &a)
	{ m_matrix = a.matrix(); }

	template <typename Derived>
	TTransform(const Eigen::MatrixBase<Derived> &M)
	{
		assert(M.cols()==4 and M.rows()==4);
		// XXX: Check singularity of rotation part
		for(int j=0; j<4; ++j) {
			for (int i=0; i<4; ++i) {
				m_matrix(i,j) = M(i,j);
		}}
	}

	TTransform(
		const double X, const double Y, const double Z,
		const double QX, const double QY, const double QZ, const double QW);

	TTransform(
		const double X, const double Y, const double Z,
		const double roll, const double pitch, const double yaw);

	static TTransform from_XYZ_RPY
		(const Eigen::Vector3d &pos,
		double roll=0, double pitch=0, double yaw=0);

	static TTransform from_Pos_Quat
		(const Eigen::Vector3d &pos,
			const Eigen::Quaterniond &ori
				=Eigen::Quaterniond::Identity());

	static TTransform from_R_t
		(const Eigen::Vector3d &t, const Eigen::Matrix3d &R);

	inline const Vector3d position() const
	{ return this->translation(); }

	inline const TQuaternion orientation() const
	{ return TQuaternion(this->rotation()); }

	std::string
	str (bool simple=false) const;

	void
	displacement (const TTransform &other, double &linear, double &angular) const;

	static TTransform interpolate(const TTransform &T1, const TTransform &T2, const double ratio);

	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		ar << boost::serialization::make_array(data(), 16);
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		ar >> boost::serialization::make_array(data(), 16);
	}

	inline double &x()
	{ return translation().x(); }

	inline double &y()
	{ return translation().y(); }

	inline double &z()
	{ return translation().z(); }

	const double x() const
	{ return position().x(); }

	const double y() const
	{ return position().y(); }

	const double z() const
	{ return position().z(); }

	const double qx() const
	{ return orientation().x(); }

	const double qy() const
	{ return orientation().y(); }

	const double qz() const
	{ return orientation().z(); }

	const double qw() const
	{ return orientation().w(); }

	bool isValid() const;

	TTransform shift(const Eigen::Vector3d &vs) const;
	inline TTransform shift(const double &x, const double &y, const double &z) const
	{ return shift(Eigen::Vector3d(x, y, z)); }

	TTransform rotate(const double roll, const double pitch=0, const double yaw=0) const;

	// Used for `transformation per time unit' and multiplication of velocity against time
	TTransform operator * (const double &div) const;
	TTransform operator / (const double &div) const;

	inline const TTransform operator * (const TTransform& other) const
	{ return Eigen::Affine3d::operator*(other); }

	inline const Eigen::Vector3d operator * (const Eigen::Vector3d &V) const
	{ return Eigen::Affine3d::operator*(V); }

	inline Eigen::Isometry3d toIsometry() const
	{
		Eigen::Isometry3d m;
		m.translation() = translation();
		m.linear() = rotation();
		return m;
	}

	/*
	 * XXX: Dubious
	 */
	inline static TTransform
	fromIsometry(const Eigen::Isometry3d &ti)
	{
		TTransform mti;
		mti.translation() = ti.translation();
		mti.linear() = ti.rotation();
		return mti;
	}

/*
	TTransform operator / (const tduration &td) const
	{ return *this / toSeconds(td); }
*/

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};

typedef TTransform Pose;


/*
 * Debugging vectors and quaternions by output to string
 */
std::string dumpVector(const Eigen::Vector3d &v);

std::string dumpVector(const Eigen::Quaterniond &v);

std::string dumpVector(const TTransform &P);


} /* namespace Vmml */

#endif /* VMML_CORE_POSE_H_ */
