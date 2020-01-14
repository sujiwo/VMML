/*
 * Trajectory.h
 *
 *  Created on: Dec 5, 2018
 *      Author: sujiwo
 */

#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_


#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include <ros/time.h>

#include "utilities.h"
#include "Pose.h"


namespace Vmml {

struct PoseStamped : public Pose
{
	ptime timestamp;

	PoseStamped():
		Pose()
	{ timestamp = ros::Time(0).toBoost(); }

	PoseStamped(const Pose &p, const ptime &t=unixTime0)
	{
		m_matrix = p.matrix();
		timestamp = t;
	}

	inline PoseStamped (const Eigen::Vector3d &p, const Quaterniond &q, const ptime &t=unixTime0)
	{
		m_matrix = Pose::from_Pos_Quat(p, q).matrix();
		timestamp = t;
	}

	PoseStamped operator* (const Pose &t) const;

	inline double timeSecond () const
	{ return toSeconds(timestamp); }

	static PoseStamped interpolate(
		const PoseStamped &p1,
		const PoseStamped &p2,
		const ptime &t);

	static
	Quaterniond
	extrapolate(const PoseStamped &p1, const PoseStamped &p2, const decltype(PoseStamped::timestamp) &tx);

	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		// Eigen matrix
		ar << boost::serialization::base_object<Pose>(*this);
		ar << timestamp;
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		// Eigen matrix
		ar >> boost::serialization::base_object<Pose>(*this);
		ar >> timestamp;
	}

	std::string dump() const;

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};


/*
 * Encodes velocity in free space, broken into linear and angular parts
 */
struct Twist
{
	// Zero
	Twist() :
		linear(Eigen::Vector3d::Zero()),
		angular(Eigen::Vector3d::Zero())
	{}

	// calculate twist from two poses
	Twist(const PoseStamped &p1, const PoseStamped &p2);

	// calculate twist from two poses with time difference
	Twist(const Pose &p1, const Pose &p2, const double &tseconds);

	// calculate twist from transformation and time difference
	Twist(const TTransform &tf, const double &tseconds);

	TTransform displacement(const tduration &td) const;
	TTransform displacement(const double &seconds) const;

	inline TTransform operator*(const double &seconds) const
	{ return displacement(seconds); }

	inline TTransform operator*(const tduration &td) const
	{ return displacement(td); }

	PoseStamped extrapolate(const tduration &td) const;
	PoseStamped extrapolate(const double &seconds) const;

	Eigen::Vector3d linear, angular;
	PoseStamped anchor;

	friend class boost::serialization::access;
private:
	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		ar << boost::serialization::make_array(linear.data(), 3);
		ar << boost::serialization::make_array(angular.data(), 3);
		ar & anchor;
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		ar >> boost::serialization::make_array(linear.data(), 3);
		ar >> boost::serialization::make_array(angular.data(), 3);
		ar & anchor;
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};


class Trajectory : public std::vector<PoseStamped>
{
public:

	friend class boost::serialization::access;

	void push_back(const PoseStamped &);

	// Return nearest element at provided time
	PoseStamped at(const ptime&) const;

	PoseStamped at(const int idx) const
	{ return std::vector<PoseStamped>::at(idx); }

	PoseStamped interpolate (const ptime&, uint32_t *i0=nullptr, uint32_t *i1=nullptr) const;

	PoseStamped extrapolate (const ptime&) const;

	Trajectory subset(const ptime &start, const ptime &stop) const;

	// Output this trajectory to a file, or to stdout when empty filename is given
	bool dump(const std::string &filename="") const;

	/*
	 * Estimate velocities
	 */
	const Twist getVelocityAt (const int idx) const;

	/*
	 * Estimate velocities at this time.
	 * Integer position is taken as nearest before t
	 */
	inline const Twist getVelocityAt (const ptime &t) const
	{ return getVelocityAt(find_lower_bound(t)); }

	uint32_t
	find_lower_bound(const ptime&) const;

	bool isInside(const ptime &t) const;

	void transform(const TTransform &tx);

	// Create new trajectory like this, but the first position is origin
	// if onlyPosition is set to true, returned trajectory has starting position at origin
	// but not changing orientation
	Trajectory setToOrigin(bool onlyPosition=false) const;

	double getElapsedDistance (const uint frontPos, const uint backPos=0) const;

private:
	typedef std::vector<PoseStamped> Parent;

protected:
	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{ ar & boost::serialization::base_object<Parent>(*this);}

};

}		// namespace Vmml

#endif /* _TRAJECTORY_H_ */
