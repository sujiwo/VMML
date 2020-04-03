/*
 * ROSConnector.cpp
 *
 *  Created on: Apr 3, 2020
 *      Author: sujiwo
 */

#include <iostream>
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
#include <ROSConnector.h>


using namespace std;

namespace Vmml {
namespace Mapper {


#define copyMatToArray(mat,target) \
	assert(mat.rows()*mat.cols()==target.size()); \
	for (uint i=0; i<mat.rows(); ++i) { \
		for (uint j=0; j<mat.cols(); j++) { \
			target[i*mat.rows()+j] = mat(i,j); \
		} \
	}

template<class T, std::size_t N, typename Scalar>
void copyToArray(const Eigen::MatrixBase<Scalar> &mat, boost::array<T,N> &target)
{
	copyMatToArray(mat, target);
}

template<class T, typename Scalar>
void copyToArray(const Eigen::MatrixBase<Scalar> &mat, vector<T> &target)
{
	target.resize(mat.cols()*mat.rows());
	copyMatToArray(mat, target);
}

sensor_msgs::CameraInfo::Ptr createCameraInfoMsg (const CameraPinholeParams &c)
{
	sensor_msgs::CameraInfo::Ptr info(new sensor_msgs::CameraInfo);
	info->distortion_model = "plump_bob";
	info->width = c.width;
	info->height = c.height;
	auto K = c.toMatrix3();
	auto P = c.toMatrix();
	copyToArray(K, info->K);
	copyToArray(c.distortionCoeffs, info->D);
	copyToArray(P, info->P);

	return info;
}


ROSConnector::ROSConnector(int argc, char *argv[], const std::string &nodeName)
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
}

ROSConnector::~ROSConnector()
{
}

int
ROSConnector::createImagePublisher(const std::string &topic, CameraPinholeParams cameraParams)
{
	// Do nothing when ROS is disabled
	if (rosDisabled)
		return -1;

	string imageTopic = topic + "/image_raw",
		cameraInfoTopic = topic + "/camera_info";

	ImagePublisher ip;
	ip.topic=topic;
	ip.cameraParams = cameraParams;
	ip.transport.reset(new image_transport::ImageTransport(*hdl));
	ip.publisher = ip.transport->advertise(imageTopic, 1);
	ip.cameraInfoPublisher = hdl->advertise<sensor_msgs::CameraInfo>(cameraInfoTopic, 1);

	imgPublishers.push_back(ip);
	return imgPublishers.size()-1;
}

void
ROSConnector::publishImage(const cv::Mat &imgSrc, int publisherId, ros::Time t) const
{
	if (rosDisabled==true)
		return;

	if (t==ros::TIME_MIN)
		t = ros::Time::now();

	auto imgPub = imgPublishers.at(publisherId);
	cv::Mat img;
	if (imgPub.cameraParams.width!=-1 and imgPub.cameraParams.width!=imgSrc.cols) {
		cv::resize(imgSrc, img, cv::Size(imgPub.cameraParams.width, imgPub.cameraParams.height));
	}
	else img = imgSrc;

	cv_bridge::CvImage cvImg;
	if (img.channels()==3)
		cvImg.encoding = sensor_msgs::image_encodings::BGR8;
	else
		cvImg.encoding = sensor_msgs::image_encodings::MONO8;

	cvImg.image = img;
	cvImg.header.stamp = t;
	imgPub.publisher.publish(cvImg.toImageMsg());

	if (imgPub.cameraParams.width!=-1) {
		auto info = createCameraInfoMsg(imgPub.cameraParams);
		info->header.stamp = t;
//		imgPub.cameraInfoPublisher.publish(info);
	}
}


} /* namespace Mapper */
} /* namespace Vmml */
