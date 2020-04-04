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

sensor_msgs::CameraInfo
ROSConnector::createCameraInfoMsg (const CameraPinholeParams &c)
{
	sensor_msgs::CameraInfo info;
	info.distortion_model = "plump_bob";
	info.width = c.width;
	info.height = c.height;
	auto K = c.toMatrix3();
	auto P = c.toMatrix();
	copyToArray(K, info.K);
	copyToArray(c.distortionCoeffs, info.D);
	copyToArray(P, info.P);
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
	imageTransport.reset(new image_transport::ImageTransport(*hdl));
}

ROSConnector::~ROSConnector()
{
}

int
ROSConnector::createImagePublisher(const std::string &basetopic, CameraPinholeParams cameraParamsInp)
{
	// Do nothing when ROS is disabled
	if (rosDisabled)
		return -1;

	ImagePublisher ip;
	ip.publisher = imageTransport->advertiseCamera(basetopic, 1);
	ip.cameraParams = createCameraInfoMsg(cameraParamsInp);
	imgPublishers.push_back(ip);
	auto id=imgPublishers.size()-1;
	return id;
}

void
ROSConnector::setCameraParam(int publisherId, const CameraPinholeParams &cam)
{
	if (cam.width!=-1) {
		imgPublishers.at(publisherId).cameraParams = createCameraInfoMsg(cam);
	}
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
	imgPub.publisher.publish(*cvImg.toImageMsg(), imgPub.cameraParams, t);
}


tf::Transform
ROSConnector::createPose(const Pose &ps)
{
	tf::Transform fPose;
	auto pos = ps.position();
	auto orn = ps.orientation();
	fPose.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
	fPose.setRotation(tf::Quaternion(orn.x(), orn.y(), orn.z(), orn.w()));

	return fPose;
}


ROSConnector::PublisherId
ROSConnector::createPosePublisher(const std::string &parentFrame, const std::string &myFrame)
{
	if (rosDisabled==true)
		return -1;

	PosePublisher pz;
	pz.parentFrame = parentFrame;
	pz.myFrame = myFrame;

	posesPub.push_back(pz);
	return posesPub.size()-1;
}


void
ROSConnector::publishPose(PublisherId pId, const Vmml::Pose &pose, ros::Time t)
const
{
	if (rosDisabled==true)
		return;

	if (t==ros::TIME_MIN)
		t = ros::Time::now();

	auto pz = posesPub.at(pId);
	auto tfp = createPose(pose);
	tf::StampedTransform kfStampedPose(tfp, t, pz.parentFrame, pz.myFrame);
	pz.sendTransform(kfStampedPose);
}


geometry_msgs::Pose
ROSConnector::createGeomPose(const Pose &p)
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


ROSConnector::PublisherId
ROSConnector::createTrajectoryPublisher(const std::string &topic, const std::string &originFrame)
{
	if (rosDisabled==true)
		return -1;

	TrajectoryPublisher trackpub;
	trackpub.pub = hdl->advertise<geometry_msgs::PoseArray>(topic, 1);
	trackpub.originFrameName = originFrame;
	trackPublishers.push_back(trackpub);
	return trackPublishers.size()-1;
}


void ROSConnector::publishTrajectory(const std::vector<Pose> &track, ros::Time t, PublisherId id) const
{
	if (rosDisabled==true)
		return;

	if (t==ros::TIME_MIN)
		t = ros::Time::now();

	auto &pub = trackPublishers.at(id);

	geometry_msgs::PoseArray allKeyframes;
	allKeyframes.header.stamp = t;
	allKeyframes.header.frame_id = pub.originFrameName;

	for (auto &pf: track) {
		auto kfp = createGeomPose(pf);
		allKeyframes.poses.push_back(kfp);
	}
	pub.pub.publish(allKeyframes);
}

} /* namespace Mapper */
} /* namespace Vmml */
