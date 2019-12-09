/*
 * RVizConnector.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/master.h>
#include "RVizConnector.h"

using namespace std;
using namespace Eigen;


const string originFrame = "world";
const string imageTopicName = "imageframe";
const string lidarScanTopicName = "lidarframe";
const string visionMapPcTopicName = "visionpcl";


namespace Vmml {
namespace Mapper {


tf::Transform createPose(const BaseFrame &f)
{
	tf::Transform fPose;
	auto pos = f.pose().position();
	auto orn = f.pose().orientation();
	fPose.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
	fPose.setRotation(tf::Quaternion(orn.x(), orn.y(), orn.z(), orn.w()));

	return fPose;
}


geometry_msgs::Pose createGeomPose(const Pose &p)
{
	geometry_msgs::Pose px;
	px.position.x = p.x();
	px.position.y = p.y();
	px.position.z = p.z();
	px.orientation.w = p.qw();
	px.orientation.x = p.qx();
	px.orientation.y = p.qy();
	px.orientation.z = p.qz();
	return px;
}


geometry_msgs::Pose createGeomPose(const BaseFrame &f)
{
	return createGeomPose(f.pose());
}


sensor_msgs::CameraInfo createCameraInfo(const CameraPinholeParams &cam)
{
	sensor_msgs::CameraInfo cmInfo;
	cmInfo.width = cam.width;
	cmInfo.height = cam.height;
	cmInfo.K[0] = cam.fx;
	cmInfo.K[2] = cam.cx;
	cmInfo.K[4] = cam.fy;
	cmInfo.K[5] = cam.cy;
	cmInfo.K[8] = 1.0;
	cmInfo.P[0] = cam.fx;
	return cmInfo;
}


RVizConnector::RVizConnector(int argc, char *argv[], const std::string &nodeName, const TTransform& _lidarToCam):
	lidarToCamera(_lidarToCam)
{
	/*
	 * Disable ROS connector when debugging (set __NOROS environment variable to 1)
	 */
	ros::init(argc, argv, nodeName);
	ros::Time::init();
	auto checkDebug=getenv("__NOROS");
	auto roschk = ros::master::check();
	if (checkDebug!=NULL or roschk==false) {
		cout << "ROS connector is disabled\n";
		rosDisabled = true;
		return;
	}
	else rosDisabled = false;

	hdl.reset(new ros::NodeHandle);
	imagePubTr.reset(new image_transport::ImageTransport(*hdl));
	posePubTf.reset(new tf::TransformBroadcaster);
	imagePub = imagePubTr->advertise(imageTopicName, 1);
	lidarScanPub = hdl->advertise<sensor_msgs::PointCloud2> ("lidar", 1);
	mapPointsPub = hdl->advertise<sensor_msgs::PointCloud2> ("map_points", 1);
	keyframePosePub = hdl->advertise<geometry_msgs::PoseArray> ("keyframe_pose", 1);
}


RVizConnector::~RVizConnector()
{}


void
RVizConnector::publishFrame(const Vmml::MapBuilder::TmpFrame &workFrame)
{
	if (rosDisabled==true)
		return;

	auto timestamp = ros::Time::now();

	// Pose
	const tf::Transform kfPose = createPose(workFrame);
	tf::StampedTransform kfStampedPose(kfPose, timestamp, originFrame, "camera");
	posePubTf->sendTransform(kfStampedPose);

	// Image
	cv_bridge::CvImage cvImg;
	cvImg.image = drawFrame(workFrame);
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.header.stamp = timestamp;
	imagePub.publish(cvImg.toImageMsg());
}


sensor_msgs::ImageConstPtr
RVizConnector::createImageMsgFromFrame(const BaseFrame &fr) const
{
	cv_bridge::CvImage cvImg;
	cvImg.image = fr.getImage();
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	return cvImg.toImageMsg();
}


cv::Mat
RVizConnector::drawFrame(const MapBuilder::TmpFrame &workFrame)
{
	return drawFrameWithPairList(workFrame, workFrame.prevMapPointPairs);
}


cv::Mat
RVizConnector::drawFrameWithPairList (const Vmml::BaseFrame &frame, const Matcher::PairList &featurePairs)
{
	cv::Mat buffer = frame.getImage().clone();
	for (auto &keypointId: featurePairs) {
		auto keypoint = frame.keypoint(keypointId.second);
		cv::circle(buffer, keypoint.pt, 2.0, cv::Scalar(0,255,0));
	}

	return buffer;
}


void
RVizConnector::publishFrameWithLidar(const Vmml::ImageDatabaseBuilder::IdbWorkFrame &workFrame)
{
	if (rosDisabled==true)
		return;

	currentTime = Vmml::getCurrentTime();

	BaseFrame virtFrame = workFrame;
	virtFrame.setPose(workFrame.pose() * lidarToCamera);

	auto currentKeyFrame = mMap->keyframe(workFrame.keyframeRel);
	publishBaseFrame(virtFrame, *currentKeyFrame);
	publishPointCloudLidar(*workFrame.lidarScan, workFrame.pose());

	if (workFrame.isKeyFrame==true) {
/*
		geometry_msgs::PoseStamped keyFramePose;
		keyFramePose.header.stamp = ros::Time::fromBoost(currentTime);
		keyFramePose.header.frame_id = originFrame;
		keyFramePose.pose = createGeomPose(*currentKeyFrame);
		keyframePosePub.publish(keyFramePose);
*/
		publishAllCurrentKeyFrames();
		publishPointCloudMap();
	}

	else {
		// TF Pose
		const tf::Transform kfPose = createPose(virtFrame);
		tf::StampedTransform kfStampedPose(kfPose, ros::Time::fromBoost(currentTime), originFrame, "camera");
		posePubTf->sendTransform(kfStampedPose);
	}
}


void
RVizConnector::publishBaseFrame(const Vmml::BaseFrame &frame, const Vmml::KeyFrame& relatedKeyFrame)
{
	if (rosDisabled)
		return;

	cv_bridge::CvImage cvImg;
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;

	cvImg.image = frame.getImage().clone();
	auto pointList = relatedKeyFrame.getVisibleMapPoints();
	for (auto &pt: pointList) {
		auto v = frame.project(pt->getPosition());
		cv::circle(cvImg.image, cv::Point2f(v.x(), v.y()), 2.0, cv::Scalar(0,255,0));
	}

	cvImg.header.stamp = ros::Time::fromBoost(currentTime);
	imagePub.publish(cvImg.toImageMsg());
}


void
RVizConnector::publishBaseFrame(const Vmml::BaseFrame &frame, const Matcher::PairList &featurePairs)
{
	if (rosDisabled)
		return;

	auto timestamp = ros::Time::now();

	// Pose
	const tf::Transform kfPose = createPose(frame);
	tf::StampedTransform kfStampedPose(kfPose, timestamp, originFrame, "camera");
	posePubTf->sendTransform(kfStampedPose);

	// Image
	cv_bridge::CvImage cvImg;
	cvImg.image = drawFrameWithPairList(frame, featurePairs);
	cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	cvImg.header.stamp = timestamp;
	imagePub.publish(cvImg.toImageMsg());
}


void
RVizConnector::publishPointCloudMap()
{
	auto triangulationPoints = mMap->dumpPointCloudFromMapPoints();

	sensor_msgs::PointCloud2 mapCloud;
	pcl::toROSMsg(*triangulationPoints, mapCloud);
	mapCloud.header.stamp = ros::Time::fromBoost(currentTime);
	mapCloud.header.frame_id = originFrame;

	mapPointsPub.publish(mapCloud);
}


void
RVizConnector::publishPointCloudLidar(const Vmml::ImageDatabaseBuilder::CloudT &cl, const TTransform &lidarPos)
{
	Vmml::ImageDatabaseBuilder::CloudT transformedCl;
	pcl::transformPointCloud(cl, transformedCl, lidarPos.matrix().cast<float>());

	sensor_msgs::PointCloud2 lidarCloud;
	pcl::toROSMsg(transformedCl, lidarCloud);
	lidarCloud.header.stamp = ros::Time::fromBoost(currentTime);
	lidarCloud.header.frame_id = originFrame;

	lidarScanPub.publish(lidarCloud);
}


void
RVizConnector::publishAllCurrentKeyFrames()
{
	geometry_msgs::PoseArray allKeyframes;
	allKeyframes.header.stamp = ros::Time::fromBoost(currentTime);
	allKeyframes.header.frame_id = originFrame;

	auto kfList = mMap->dumpCameraTrajectory();
	for (auto &kf: kfList) {
		auto kfp = createGeomPose(kf);
		allKeyframes.poses.push_back(kfp);
	}

	keyframePosePub.publish(allKeyframes);

}


} /* namespace Mapper */
} /* namespace Vmml */
