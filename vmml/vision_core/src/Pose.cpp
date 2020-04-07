/*
 * Pose.cpp
 *
 *  Created on: Oct 4, 2019
 *      Author: sujiwo
 */

#include "vmml/Pose.h"
#include "vmml/utilities.h"

using namespace std;
using namespace Eigen;


#define dPrecision 3


namespace Vmml {


const ptime epoch(boost::gregorian::date(1970,1,1));


class VStream: public std::stringstream
{
public:
VStream()
{
	setf(ios::fixed, ios::floatfield);
	precision(dPrecision);
}
};


string
dumpVector(const Vector3d &v)
{
	VStream s;
	s << v.x() << " " << v.y() << " " << v.z();
	return s.str();
}


string
dumpVector(const Quaterniond &v)
{
	VStream s;
	s << v.x() << " " << v.y() << " " << v.z() << ' ' << v.w();
	return s.str();
}


std::string
dumpVector(const TTransform &P)
{
	Vector3d xyz = P.position();
	TQuaternion qxyz = P.orientation();
	Vector3d RPY = quaternionToRPY(qxyz);

	VStream s;
	s << "XYZ: " << dumpVector(xyz) << endl;
	s << "RPY (rad): " << dumpVector(RPY) << endl;
	s << "QXYZW: " << dumpVector(qxyz) << endl;

	return s.str();
}


Quaterniond fromRPY (double roll, double pitch, double yaw)
{
	roll /= 2.0;
	pitch /= 2.0;
	yaw /= 2.0;
	double
		ci = cos(roll),
		si = sin(roll),
		cj = cos(pitch),
		sj = sin(pitch),
		ck = cos(yaw),
		sk = sin(yaw),
		cc = ci*ck,
		cs = ci*sk,
		sc = si*ck,
		ss = si*sk;
	Quaterniond q;
	q.x() = cj*sc - sj*cs;
	q.y() = cj*ss + sj*cc;
	q.z() = cj*cs - sj*sc;
	q.w() = cj*cc + sj*ss;
	q.normalize();
	return q;
}


Vector3d quaternionToRPY (const Quaterniond &q)
{
	Eigen::Matrix3d m = q.toRotationMatrix();

	double cy = sqrt(m(0,0)*m(0,0) + m(1,0)*m(1,0));
	double ax, ay, az;
	if (cy > std::numeric_limits<double>::epsilon()*4.0) {
		ax = atan2( m(2, 1),  m(2, 2));
		ay = atan2(-m(2, 0),  cy);
		az = atan2( m(1, 0),  m(0,0));
	}
	else {
        ax = atan2(-m(1, 2),  m(1, 1));
        ay = atan2(-m(2, 0),  cy);
        az = 0.0;
	}
	return Vector3d(ax, ay, az);
}


TTransform::TTransform(
	const double X, const double Y, const double Z,
	const double QX, const double QY, const double QZ, const double QW) :
		Affine3d(
			Eigen::Translation3d(X,Y,Z) * Eigen::Quaterniond(QW, QX, QY, QZ))
{}

TTransform::TTransform(
	const double X, const double Y, const double Z,
	const double roll, const double pitch, const double yaw) :
		Affine3d(
			Eigen::Translation3d(X,Y,Z) * fromRPY(roll, pitch, yaw))
{}



TTransform
TTransform::from_XYZ_RPY (
	const Eigen::Vector3d &pos,
	double roll, double pitch, double yaw)
{
	Quaterniond q = fromRPY(roll, pitch, yaw);
	return TTransform::from_Pos_Quat(pos, q);
}


TTransform
TTransform::from_Pos_Quat(const Vector3d &pos, const Quaterniond &orient)
{
	Affine3d t;
	t = Eigen::Translation3d(pos) * orient;
	return t;
}


TTransform
TTransform::from_R_t
(const Eigen::Vector3d &t, const Eigen::Matrix3d &R)
{
	TTransform T;
	T.m_matrix.block<3,3>(0,0) = R;
	T.m_matrix.block<3,1>(0,3) = t;
	return T;
}


#include <sstream>

string
TTransform::str(bool simple) const
{
	stringstream ss;
	auto P = position();
	auto Q = orientation();

	if (simple==false) {
		ss << "x=" << P.x()
			<< " y=" << P.y()
			<< " z=" << P.z()
			<< ", qx=" << Q.x()
			<< " qy=" << Q.y()
			<< " qz=" << Q.z()
			<< " qw=" << Q.w();
	}
	else {
		ss << "XYZ Q(XYZW): ";
		ss << P.x() << ' '
			<< P.y() << ' '
			<< P.z() << ' ';
		ss << Q.x() << ' '
			<< Q.y() << ' '
			<< Q.z() << ' '
			<< Q.w();
	}

	return ss.str();
}


Vector4d vectorFromQuaternion (const Quaterniond &q)
{
	return Vector4d(q.x(), q.y(), q.z(), q.w());
}


void
TTransform::displacement (
	const TTransform &other,
	double &linear, double &angular) const
{
	TTransform T = this->inverse() * other;
	linear = T.translation().norm();
	angular = 2*acos(abs(T.qw()));
}


TTransform
TTransform::interpolate(const TTransform &T1, const TTransform &T2, const double ratio)
{
	assert (0<=ratio and ratio<=1);

	Vector3d
		p1 = T1.position(),
		p2 = T2.position(),
		p3;
	Quaterniond
		q1 = T1.orientation(),
		q2 = T2.orientation(),
		q3;

	p3 = p1 + ratio * (p2 - p1);
	q3 = q1.slerp(ratio, q2);
	return TTransform::from_Pos_Quat(p3, q3);
}


bool TTransform::isValid() const
{
	auto P = position();

	if (isnan(P.x()) or isnan(P.y()) or isnan(P.z()))
		return false;

	return true;
}


TTransform
TTransform::shift(const Eigen::Vector3d &vs)
const
{
	TTransform tmp = TTransform::from_XYZ_RPY(vs);

	TTransform S = (*this) * tmp;
	return S;
}


TTransform
TTransform::rotate(const double roll, const double pitch, const double yaw)
const
{
	TTransform tmp = TTransform::from_XYZ_RPY(Vector3d::Zero(), roll, pitch, yaw);
	return (*this) * tmp;
}


TQuaternion TQuaternion::operator * (const double &mul) const
{
	auto euler = quaternionToRPY(*this);
	euler *= mul;

	return fromRPY(euler.x(), euler.y(), euler.z());
}


TQuaternion TQuaternion::operator / (const double &div) const
{
	auto euler = quaternionToRPY(*this);
	euler /= div;

	return fromRPY(euler.x(), euler.y(), euler.z());
}


TTransform TTransform::operator / (const double &div) const
{
	auto qn = orientation() / div;
	auto pn = position() / div;
	return from_Pos_Quat(pn, qn);
}


TTransform TTransform::operator * (const double &mul) const
{
	auto qn = orientation() * mul;
	auto pn = position() * mul;
	return from_Pos_Quat(pn, qn);
}


std::string
TTransform::dump() const
{
	std::stringstream s;
	Vector3d v = position();
	s << v.x() << " " << v.y() << " " << v.z() << " ";
	Quaterniond q = orientation();
	s << q.x() << " " << q.y() << " " << q.z() << ' ' << q.w();

	return s.str();
}

} /* namespace Vmml */
