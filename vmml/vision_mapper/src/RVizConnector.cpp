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
#include <sensor_msgs/image_encodings.h>
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


RVizConnector::RVizConnector(int argc, char *argv[], const std::string &nodeName)
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

	auto currentKeyFrame = mMap->keyframe(workFrame.keyframeRel);
	publishBaseFrame(*currentKeyFrame, workFrame.featureMatchesFromLastAnchor);
	publishPointCloudLidar(*workFrame.lidarScan, workFrame.pose());

	if (mMap) {
		publishPointCloudMap();
	}
}


void
RVizConnector::publishBaseFrame(const Vmml::BaseFrame &frame, const Matcher::PairList &featurePairs)
{
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
	mapCloud.header.stamp = ros::Time::now();
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
	lidarCloud.header.stamp = ros::Time::now();
	lidarCloud.header.frame_id = originFrame;

	lidarScanPub.publish(lidarCloud);

}


} /* namespace Mapper */
} /* namespace Vmml */
