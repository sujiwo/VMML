/*
 * RVizConnector.h
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_RVIZCONNECTOR_H_
#define VMML_MAPPER_RVIZCONNECTOR_H_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include "BaseFrame.h"
#include "VisionMap.h"


namespace Vmml {
namespace Mapper {


class RVizConnector {
public:
	RVizConnector(int argc, char *argv[], const std::string &nodeName);
	virtual ~RVizConnector();

	void setMap(const VisionMap::Ptr &vmap);

	void publishFrame(const BaseFrame &fr);

	void publishKeyFrame(const KeyFrame &kf);

	void publishPointcloud();

protected:
	std::shared_ptr<ros::NodeHandle> hdl;
	VisionMap::Ptr mMap;

	std::shared_ptr<image_transport::ImageTransport> imagePubTr;
	image_transport::Publisher imagePub;
	std::shared_ptr<tf::TransformBroadcaster> posePubTf;

	sensor_msgs::ImageConstPtr createImageMsgFromFrame(const BaseFrame &fr) const;

	static cv::Mat drawKeyFrame(const KeyFrame &kf);
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_RVIZCONNECTOR_H_ */
