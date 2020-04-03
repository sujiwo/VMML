/*
 * ROSConnector.h
 *
 *  Created on: Apr 3, 2020
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_ROSCONNECTOR_H_
#define VMML_MAPPER_ROSCONNECTOR_H_

#include <string>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/publisher.h>
#include "vmml/CameraPinholeParams.h"
#include "vmml/Pose.h"
#include "vmml/Trajectory.h"


namespace Vmml {
namespace Mapper {

class ROSConnector {
public:
	typedef int PublisherId;

	ROSConnector(int argc, char *argv[], const std::string &nodeName);
	virtual ~ROSConnector();

	struct PosePublisher {

	};

	/*
	 * Images.
	 * These functions automate publishing of image and corresponding cameraInfo, if any.
	 */
	struct ImagePublisher {
		std::shared_ptr<image_transport::ImageTransport> transport;
		image_transport::Publisher publisher;
		ros::Publisher cameraInfoPublisher;
		std::string topic;
		sensor_msgs::CameraInfo cameraParams;
	};

	// create publisher
	int createImagePublisher(const std::string &topic, CameraPinholeParams cameraParams=CameraPinholeParams());

	void setCameraParam(int publisherId, const CameraPinholeParams &cam);

	void publishImage(const cv::Mat &img, int publisherId, ros::Time t=ros::TIME_MIN) const;

	/*
	 * Trajectory
	 */
	struct TrajectoryPublisher {
		std::string topic;
		ros::Publisher publisher;
	};
	int createTrajectoryPublisher(const std::string &topic);
	void publishTrajectory(const std::vector<Pose> &track, const ros::Time=ros::TIME_MIN, int publisherId=0) const;
	void publishTrajectory(const Trajectory &track, const ros::Time=ros::TIME_MIN, int publisherId=0) const;

	/*
	 * Point Cloud
	 */
	struct PointCloudPublisher : public TrajectoryPublisher {};
	int createPointCloudPublisher(const std::string &topic);
	void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloudPt, int publisherId=0) const;


protected:
	bool rosDisabled;

	std::shared_ptr<ros::NodeHandle> hdl;

	std::vector<ImagePublisher> imgPublishers;
	std::vector<TrajectoryPublisher> trackPublishers;
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_ROSCONNECTOR_H_ */
