/*
 * recognizer_srv.cpp
 *
 *  Created on: Sep 29, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "IncrementalBoW.h"
#include "ProgramOptionParser.h"
#include "place_recognizer/place_recognizer.h"


class RecognizerService
{
public:
RecognizerService(ros::NodeHandle &n)
{

}

private:
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "place_recognizer");
	ros::NodeHandle node;
	RecognizerService srv(node);

	ros::spin();

	return 0;
}
