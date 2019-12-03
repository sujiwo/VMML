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
#include "RVizConnector.h"

using namespace std;
using namespace Eigen;


const string originFrame = "world";


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
	auto checkDebug=getenv("__NOROS");
	if (checkDebug!=NULL) {
		rosDisabled = true;
		return;
	}
	else rosDisabled = false;

	ros::init(argc, argv, nodeName);
	hdl.reset(new ros::NodeHandle);
	imagePubTr.reset(new image_transport::ImageTransport(*hdl));
	posePubTf.reset(new tf::TransformBroadcaster);
	imagePub = imagePubTr->advertise("imageframe", 1);
}


RVizConnector::~RVizConnector() {
	// TODO Auto-generated destructor stub
}


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

	publishBaseFrame(workFrame);

	if (mMap) {
		auto triangulationPoints = mMap->dumpPointCloudFromMapPoints();
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


} /* namespace Mapper */
} /* namespace Vmml */
