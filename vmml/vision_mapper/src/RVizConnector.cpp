/*
 * RVizConnector.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "RVizConnector.h"


namespace Vmml {
namespace Mapper {

RVizConnector::RVizConnector(int argc, char *argv[], const std::string &nodeName)
{
	ros::init(argc, argv, nodeName);
	hdl.reset(new ros::NodeHandle);
	imagePubTr.reset(new image_transport::ImageTransport(*hdl));
	imagePub = imagePubTr->advertise("imageframe", 1);
}


RVizConnector::~RVizConnector() {
	// TODO Auto-generated destructor stub
}


void
RVizConnector::publishFrame(const BaseFrame &fr)
{
/*
	cv_bridge::CvImage cvImg;
	cvImg.image = fr.getImage();
	imagePub.publish(cvImg.toImageMsg());
*/
}


void
RVizConnector::publishKeyFrame(const KeyFrame &kf)
{
	cv_bridge::CvImage cvImg;
	cvImg.image = kf.getImage();
	imagePub.publish(createImageMsgFromFrame(kf));
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
RVizConnector::drawKeyFrame(const KeyFrame &k)
{
	cv::Mat buffer = k.getImage().clone();
}


} /* namespace Mapper */
} /* namespace Vmml */
