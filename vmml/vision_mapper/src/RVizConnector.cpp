/*
 * RVizConnector.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include <cv_bridge/cv_bridge.h>
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
	cv_bridge::CvImage cvImg;
	cvImg.image = fr.getImage();
	imagePub.publish(cvImg.toImageMsg());
}

} /* namespace Mapper */
} /* namespace Vmml */
