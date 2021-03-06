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
#include <pcl_ros/publisher.h>
#include "vmml/BaseFrame.h"
#include "vmml/Matcher.h"
#include "vmml/VisionMap.h"
#include "vmml/ImageDatabaseBuilder.h"


namespace Vmml {
namespace Mapper {


class RVizConnector {
public:
	RVizConnector(int argc, char *argv[], const std::string &nodeName, const TTransform& _lidarToCam=TTransform::Identity());
	virtual ~RVizConnector();

	inline void setMap(const VisionMap::Ptr &vmap)
	{ mMap = vmap; }

//	void publishFrame(const Vmml::MapBuilder::TmpFrame &workFrame);

//	void publishFrameWithLidar(const Vmml::ImageDatabaseBuilder::IdbWorkFrame &workFrame);

	void publishBaseFrame(const Vmml::BaseFrame &frame, const Matcher::PairList &featurePairs=Matcher::PairList());

	void publishPlainBaseFrame(const Vmml::BaseFrame &frame);

	void publishImage(const cv::Mat &img, const ros::Time &t=ros::Time(0));

	void publishPose(const Pose &camPose, const ros::Time &t);

	void publishKeyPointsInFrame(const Vmml::BaseFrame &frame);

	inline bool isRosUsed() const
	{ return !rosDisabled; }

	void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const ros::Time &t);

	std::shared_ptr<ros::NodeHandle>
	getNodeHandle()
	{ return hdl; }

	void setImageTopicName(const std::string &s);

	inline void setCameraFrameName(const std::string &c)
	{ cameraFrameName = c; }

	void publishTrajectory(const std::vector<Vmml::Pose> &vs, const ros::Time &t);
	void publishTrajectory(const Trajectory &vs, const ros::Time &t)
	{ return publishTrajectory(vs.toVectorPose(), t); }

protected:
	std::shared_ptr<ros::NodeHandle> hdl;
	VisionMap::Ptr mMap;

	// Publishers
	std::shared_ptr<image_transport::ImageTransport> imagePubTr;
	image_transport::Publisher imagePub;
	std::shared_ptr<tf::TransformBroadcaster> posePubTf;
	ros::Publisher lidarScanPub, mapPointsPub, keyframePosePub;

	sensor_msgs::ImageConstPtr createImageMsgFromFrame(const BaseFrame &fr) const;

//	static cv::Mat drawFrame(const MapBuilder::TmpFrame &workFrame);

	static cv::Mat drawFrameWithPairList (const Vmml::BaseFrame &frame, const Matcher::PairList &featurePairs=Matcher::PairList());

	void publishBaseFrame(const Vmml::BaseFrame &frame, const Vmml::KeyFrame& relatedKeyFrame);

	void publishPointCloudMap();

//	void publishPointCloudLidar(const Vmml::ImageDatabaseBuilder::CloudT &cl, const TTransform &lidarPos);

	void publishAllCurrentKeyFrames();

	bool rosDisabled;

	// Transforms
	TTransform lidarToCamera;

	ptime currentTime;

	std::string imageTopicName = "imageframe";
	std::string cameraFrameName = "camera";
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_RVIZCONNECTOR_H_ */
