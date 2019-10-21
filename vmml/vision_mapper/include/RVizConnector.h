/*
 * RVizConnector.h
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_RVIZCONNECTOR_H_
#define VMML_MAPPER_RVIZCONNECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>


namespace Vmml {
namespace Mapper {

class RVizConnector {
public:
	RVizConnector(int argc, char *argv[]);
	virtual ~RVizConnector();

protected:
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_RVIZCONNECTOR_H_ */
