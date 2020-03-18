/*
 * imagelidar_projection.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: sujiwo
 *
 *  Projection test for Lidar to image
 */

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/frustum_culling.h>
#include "vmml/BaseFrame.h"
#include "ProgramOptions.h"
#include "RVizConnector.h"


using namespace std;
using namespace Vmml;
using namespace Vmml::Mapper;

using Eigen::Matrix4f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;


class LidarImageFrame: public BaseFrame
{
public:

	static vector<Eigen::Vector2f> projectPointCloud(
		const CameraPinholeParams &camera,
		const TTransform& lidarToCamera,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
		float cutDistance=200.0)
	{
		vector<Vector2f> project2;

		pcl::FrustumCulling<pcl::PointXYZ> frustum;

		frustum.setInputCloud(cloud);
		frustum.setNearPlaneDistance(0.5);
		frustum.setFarPlaneDistance(cutDistance);
		frustum.setHorizontalFOV(camera.getHorizontalFoV() * 180 / M_PI);
		frustum.setVerticalFOV(camera.getVerticalFoV() * 180 / M_PI);

		Matrix4f cam2robot;
		cam2robot <<
				0, 0, 1, 0,
	            0,-1, 0, 0,
	            1, 0, 0, 0,
	            0, 0, 0, 1;
		Matrix4f poseFilt = lidarToCamera.cast<float>() * cam2robot;
		frustum.setCameraPose(poseFilt);

		pcl::PointCloud<pcl::PointXYZ> filtered;
		frustum.filter(filtered);

		project2.resize(filtered.size());
		// Must use this matrix data type (dont use auto)
		Eigen::Matrix<float,3,4> camMat4 = camera.toMatrix().cast<float>() * BaseFrame::createExternalParamMatrix4(lidarToCamera).cast<float>();
		for (uint64 i=0; i<filtered.size(); ++i) {
			auto pp = filtered.at(i);
			Vector4f pv(pp.x, pp.y, pp.z, 1);
			Vector3f pj = camMat4 * pv;
			pj /= pj[2];
			project2[i] = pj.hnormalized();
		}

		return project2;
	}

	static cv::Mat
	drawPointCloud(
		const cv::Mat &imageSrc,
		const CameraPinholeParams &camera,
		const TTransform& lidarToCamera,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
		float cutDistance=200.0)
	{
		cv::Mat imageRes = imageSrc.clone();
		auto projRes = projectPointCloud(camera, lidarToCamera, cloud, cutDistance);

		uint numDrawn = 0;
		for (auto &_pt2: projRes) {
			cv::Point2f pt2(_pt2.x(), _pt2.y());
			if ((pt2.x<0 or pt2.x>=imageSrc.cols) or (pt2.y<0 or pt2.y>=imageSrc.rows))
				continue;
			cv::circle(imageRes, pt2, 1, cv::Scalar(0,0,255), -1);
			numDrawn +=1;
		}

		return imageRes;
	}

protected:
};


int main(int argc, char *argv[])
{
	ProgramOptions projOptions;
	RVizConnector rosConn(argc, argv, "lidar_projection");
	rosConn.setImageTopicName("lidar_in_image");

	projOptions.parseCommandLineArgs(argc, argv);

	auto lidarBag = projOptions.getLidarScanBag();
	auto imageBag = projOptions.getImageBag();
	auto &imagePipe = projOptions.getImagePipeline();
	auto camera0 = projOptions.getWorkingCameraParameter();

	assert(lidarBag->getBagStartTime() >= imageBag->getBagStartTime());
	assert(lidarBag->getBagStopTime() <= imageBag->getBagStopTime());
	assert(camera0.fx!=0 and camera0.fy!=0);

	for (int fr=0; fr<lidarBag->size(); ++fr) {

		ptime lidarTm;
		double timeDiff;
		auto lidarScan = lidarBag->at<pcl::PointXYZ>(fr, &lidarTm);
		auto imageN = imageBag->getPositionAtTime(ros::Time::fromBoost(lidarTm));
		auto imageTm = imageBag->timeAt(imageN);
		auto imageMt = imageBag->at(imageN);

		imagePipe.run(imageMt, imageMt);

//		auto imageFr = BaseFrame::create(imageMt, camera0);
		cv::Mat frameProj = LidarImageFrame::drawPointCloud(imageMt, camera0, projOptions.getLidarToCameraTransform(), lidarScan);
		rosConn.publishImage(frameProj, imageTm);
	}

	return 0;
}
