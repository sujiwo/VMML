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
#include <image_transport/camera_publisher.h>
#include <pcl_ros/publisher.h>
#include "vmml/CameraPinholeParams.h"
#include "vmml/Pose.h"
#include "vmml/Trajectory.h"


namespace Vmml {
namespace Mapper {

/*
 * This class is an utility to abstract ROS publisher for:
 * 1) Image
 * 2) Pose
 * 3) Trajectory
 * 4) Point Cloud
 */

class ROSConnector {
public:
	typedef int PublisherId;

	ROSConnector(int argc, char *argv[], const std::string &nodeName, uint32_t RosInitOptions=0);
	virtual ~ROSConnector();

	inline std::shared_ptr<ros::NodeHandle> getRosHandle()
	{ return hdl; }

	inline std::shared_ptr<image_transport::ImageTransport> getImageTransport()
	{ return imageTransport; }

	/*
	 * Images.
	 * These functions automate publishing of image and corresponding cameraInfo, if any.
	 */
	struct ImagePublisher {
		image_transport::CameraPublisher publisher;
		sensor_msgs::CameraInfo cameraParams;
		std::string frameId;
		std::string topic() const { return publisher.getTopic(); }
	};
	PublisherId createImagePublisher(const std::string &topic, CameraPinholeParams cameraParams=CameraPinholeParams(), const std::string &frameIdName="");
	void setCameraParam(PublisherId publisherId, const CameraPinholeParams &cam);
	void publishImage(const cv::Mat &img, PublisherId publisherId, ros::Time t=ros::TIME_MIN) const;
	static sensor_msgs::CameraInfo createCameraInfoMsg (const CameraPinholeParams &c);

	/*
	 * Pose Publisher, implemented as TF
	 */
	struct PosePublisher: public tf::TransformBroadcaster {
		std::string parentFrame, myFrame;
	};
	PublisherId createPosePublisher(const std::string &parentFrame, const std::string &myFrame);
	void publishPose(PublisherId pId, const Vmml::Pose &pose, ros::Time t=ros::TIME_MIN) const;
	static tf::Transform createPose(const Pose &ps);

	/*
	 * Trajectory
	 */
	struct TrajectoryPublisher {
		ros::Publisher pub;
		std::string originFrameName;
		std::string topic() const { return pub.getTopic(); }
	};
	PublisherId createTrajectoryPublisher(const std::string &topic, const std::string &originFrame);
	void publishTrajectory(const std::vector<Pose> &track, ros::Time t=ros::TIME_MIN, PublisherId id=0) const;
	void publishTrajectory(const Trajectory &track, ros::Time t=ros::TIME_MIN, PublisherId id=0) const
	{ return publishTrajectory(track.toVectorPose(), t, id); }
	static geometry_msgs::Pose createGeomPose(const Pose &p);

	/*
	 * Point Cloud
	 */
	struct PointCloudPublisher : public TrajectoryPublisher {
		TTransform disposition=TTransform::Identity();
	};
	PublisherId createPointCloudPublisher(
		const std::string &topic,
		const std::string &originFrame,
		const TTransform &transfirst=TTransform::Identity());

	template<typename T>
	void publishPointCloud(const pcl::PointCloud<T> &cloudPtSrc, ros::Time t=ros::TIME_MIN, PublisherId id=0) const
	{
		pcl::PointCloud<T> transformedPc;
		pcl::transformPointCloud(cloudPtSrc, transformedPc, pcPublishers[id].disposition.matrix().cast<float>());
		sensor_msgs::PointCloud2 mapCloud;
		pcl::toROSMsg(cloudPtSrc, mapCloud);
		return doPublishPc(mapCloud, t, id);
	}


protected:
	bool rosDisabled;

	std::shared_ptr<ros::NodeHandle> hdl;

	// Image part
	std::shared_ptr<image_transport::ImageTransport> imageTransport;
	std::vector<ImagePublisher> imgPublishers;

	// Poses part
	std::vector<PosePublisher> posesPub;

	std::vector<TrajectoryPublisher> trackPublishers;

	// Point Clouds
	std::vector<PointCloudPublisher> pcPublishers;
	void doPublishPc(sensor_msgs::PointCloud2 &cld, ros::Time t=ros::TIME_MIN, PublisherId id=0) const;
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_ROSCONNECTOR_H_ */
