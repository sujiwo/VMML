/*
 * MapPoint.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_MAPPOINT_H_
#define VMML_CORE_MAPPOINT_H_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <tuple>
#include <opencv2/core.hpp>

#include "BaseFrame.h"

struct KeyMapPoint {
	const Vmml::BaseFrame &keyframe;
	int keypointIdx;
};


namespace Vmml {


int ORBDescriptorDistance(const cv::Mat &a, const cv::Mat &b);


class MapPoint {
public:

	typedef std::shared_ptr<MapPoint> Ptr;

//	MapPoint();
	MapPoint(const Eigen::Vector3d &p);
	virtual ~MapPoint();

	void createDescriptor(const std::vector<KeyMapPoint> &visibleIn);

	cv::Mat getDescriptor() const
	{ return descriptor.clone(); }

	double X() const
	{ return position.x(); }

	double Y() const
	{ return position.y(); }

	double Z() const
	{ return position.z(); }

	Eigen::Vector3d getPosition () const
	{ return position; }

	inline void setPosition (const Eigen::Vector3d &pw)
	{ position = pw; }

	mpid getId () const
	{ return id; }

	// Set by bundle adjustment routines
	bool hasBa = false;

	static Ptr create(const Eigen::Vector3d &p);

protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, Vmml::MapPoint &mappoint, const unsigned int version);


private:

	mpid id;

	Eigen::Vector3d position;

	// Best Descriptor
	cv::Mat descriptor;

	static mpid nextId;

};

} /* namespace Vmml */

#endif /* VMML_CORE_MAPPOINT_H_ */
