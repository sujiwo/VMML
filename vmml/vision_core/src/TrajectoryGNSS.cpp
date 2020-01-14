/*
 * TrajectoryGNSS.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: sujiwo
 */

#include <geodesy/utm.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <gnss/geo_pos_conv.hpp>
#include "RandomAccessBag.h"
#include "vmml/TrajectoryGNSS.h"


using namespace std;
using Vmml::Pose;
using Vmml::PoseStamped;
using Vmml::TQuaternion;
using Vmml::TTransform;


class wrong_nmea_sentence : public exception
{};

class invalid_gnss_position : public exception
{};


/*
These are expected results (without offset)

header:
  seq: 198
  stamp:
    secs: 1482726415
    nsecs: 842756032
  frame_id: "map"
pose:
  position:
    x: -18266.4001572
    y: -93879.5007049
    z: 43.0621
  orientation:
    x: -0.0742241380626
    y: 0.029065332244
    z: -0.962810799329
    w: -0.258149856645
*/

const double positionInvalid = std::numeric_limits<double>::max();


struct GnssLocalizerState
{
	double roll_=0, pitch_=0, yaw_=0;
	double orientation_time_=0, position_time_=0;
	double latitude=positionInvalid, longitude=positionInvalid, height=positionInvalid;
	ros::Time current_time_=ros::Time(0), orientation_stamp_=ros::Time(0);
	geo_pos_conv geo, last_geo;
};


std::vector<std::string> splitSentence(const std::string &string)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, ','))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}


void convertNMEASentenceToState (nmea_msgs::SentencePtr &msg, GnssLocalizerState &state)
{
	vector<string> nmea = splitSentence(msg->sentence);
	if (nmea.at(0).compare(0, 2, "QQ") == 0)
	{
		state.orientation_time_ = stod(nmea.at(3));
		state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
		state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
		state.yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
		state.orientation_stamp_ = msg->header.stamp;
	}

	else if (nmea.at(0) == "$PASHR")
	{
		state.orientation_time_ = stod(nmea.at(1));
		state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
		state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
		state.yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
	}

	else if(nmea.at(0).compare(3, 3, "GGA") == 0)
	{
		try {
			state.position_time_ = stod(nmea.at(1));
			state.latitude = stod(nmea.at(2));
			state.longitude = stod(nmea.at(4));
			state.height = stod(nmea.at(9));
			state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
		} catch (std::invalid_argument &e) {
			throw wrong_nmea_sentence();
		}
	}

	else if(nmea.at(0) == "$GPRMC")
	{
		state.position_time_ = stoi(nmea.at(1));
		state.latitude = stod(nmea.at(3));
		state.longitude = stod(nmea.at(5));
		state.height = 0.0;
		state.geo.set_llh_nmea_degrees(state.latitude, state.longitude, state.height);
	}

	else
		throw wrong_nmea_sentence();
}


PoseStamped createFromState(const GnssLocalizerState &state, TTransform worldToMap)
{
	if (state.latitude==positionInvalid or state.longitude==positionInvalid)
		throw invalid_gnss_position();

	TQuaternion q(state.roll_, state.pitch_, state.yaw_);
	Vector3d p(state.geo.y(), state.geo.x(), state.geo.z());

	// The above code transform latitude/longitude to UTM coordinate
	// (easting/northing).
	// The following line transform easting/northing
	// into some numbers that are `smaller'
	Pose pt = worldToMap * Pose::from_Pos_Quat(p, q);
	return pt;
}


namespace Vmml {

TrajectoryGNSS
TrajectoryGNSS::fromRosBag(rosbag::Bag &bag, const std::string &topicName, TTransform worldToMap, int plane_number)
{
	TrajectoryGNSS vehicleTrack;
	const double orientationTimeout = 10.0;

	RandomAccessBag nmeaBag(bag, topicName);
	if (nmeaBag.messageType() != "nmea_msgs/Sentence")
		throw runtime_error("Not GNSS bag");

	GnssLocalizerState state;
	state.geo.set_plane(plane_number);

	vehicleTrack.clear();
	uint32_t nMsg = nmeaBag.size();

	for (uint32_t ix=0; ix<nMsg; ix++) {
		cout << ix << "/" << nmeaBag.size() << "         \r";

		auto currentMessage = nmeaBag.at<nmea_msgs::Sentence>(ix);
		ros::Time current_time = currentMessage->header.stamp;

		try {
			convertNMEASentenceToState(currentMessage, state);
		} catch (const wrong_nmea_sentence &e)
		{
			continue;
		}

		if (fabs(state.orientation_stamp_.toSec() - currentMessage->header.stamp.toSec()) > orientationTimeout) {
			double dt = sqrt(pow(state.geo.x() - state.last_geo.x(), 2) + pow(state.geo.y() - state.last_geo.y(), 2));
			const double threshold = 0.2;
			if (dt > threshold) {
				// create fake orientation
				state.yaw_ = atan2(state.geo.x() - state.last_geo.x(), state.geo.y() - state.last_geo.y());
				state.roll_ = 0;
				state.pitch_ = 0;

				PoseStamped px;
				try {
					px = createFromState(state, worldToMap);
				} catch (invalid_gnss_position &e) {
					continue;
				}

				px.timestamp = current_time.toBoost();
				vehicleTrack.push_back(px);
				state.last_geo = state.geo;
				continue;
			}
		}

		double e = 1e-2;
		if (fabs(state.orientation_time_ - state.position_time_) < e) {

			PoseStamped px;
			try {
				px = createFromState(state, worldToMap);
			} catch (invalid_gnss_position &e) {
				continue;
			}

			px.timestamp = current_time.toBoost();
			vehicleTrack.push_back(px);
			continue;
		}
	}

	cout << "\nDone GNSS Trajectory generation\n";

	return vehicleTrack;
}


struct GnssLocalizerState2
{
	double roll_=0, pitch_=0, yaw_=0;
	double orientation_time_=0, position_time_=0;
	double latitude=positionInvalid, longitude=positionInvalid, height=positionInvalid;
	ros::Time current_time_=ros::Time(0), orientation_stamp_=ros::Time(0);

	geographic_msgs::GeoPoint toGeoPoint() const
	{
		geographic_msgs::GeoPoint gp;
		gp.altitude = height;
		gp.latitude = latitude;
		gp.longitude = longitude;
		return gp;
	}

	Pose createPose() const
	{
		if (latitude==positionInvalid or longitude==positionInvalid)
			throw invalid_gnss_position();

		TQuaternion q(roll_, pitch_, yaw_);
		Vector3d p(geo.northing, geo.easting, geo.altitude);

		return Pose::from_Pos_Quat(p, q);
	}

	geodesy::UTMPoint geo, last_geo;
};


TrajectoryGNSS
TrajectoryGNSS::fromRosBag2(rosbag::Bag &bag, const std::string &topicName)
{
	TrajectoryGNSS vehicleTrack;

	RandomAccessBag nmeaBag(bag, topicName);
	if (nmeaBag.messageType() != "nmea_msgs/Sentence")
		throw runtime_error("Not GNSS bag");

	GnssLocalizerState2 state;
	const double orientationTimeout = 10.0;

	for (int i=0; i<nmeaBag.size(); i++) {
		auto currentMessage = nmeaBag.at<nmea_msgs::Sentence>(i);
		ros::Time current_time = currentMessage->header.stamp;

		vector<string> nmea = splitSentence(currentMessage->sentence);

		try {

			if (nmea.at(0).compare(0, 2, "QQ") == 0)
			{
				state.orientation_time_ = stod(nmea.at(3));
				state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
				state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
				state.yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
				state.orientation_stamp_ = currentMessage->header.stamp;
			}

			else if (nmea.at(0) == "$PASHR")
			{
				state.orientation_time_ = stod(nmea.at(1));
				state.roll_ = stod(nmea.at(4)) * M_PI / 180.;
				state.pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
				state.yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
			}

			else if(nmea.at(0).compare(3, 3, "GGA") == 0)
			{
				try {
					state.position_time_ = stod(nmea.at(1));
					state.latitude = stod(nmea.at(2));
					state.longitude = stod(nmea.at(4));
					state.height = stod(nmea.at(9));
					state.geo = geodesy::UTMPoint(state.toGeoPoint());
				} catch (std::invalid_argument &e) {
					throw wrong_nmea_sentence();
				}
			}

			else if(nmea.at(0) == "$GPRMC")
			{
				state.position_time_ = stoi(nmea.at(1));
				state.latitude = stod(nmea.at(3));
				state.longitude = stod(nmea.at(5));
				state.height = 0.0;
				state.geo = geodesy::UTMPoint(state.toGeoPoint());
			}

			else
				throw wrong_nmea_sentence();

		} catch (wrong_nmea_sentence &e) { continue; }

		if (fabs(state.orientation_stamp_.toSec() - currentMessage->header.stamp.toSec()) > orientationTimeout) {
			double dt = sqrt(pow(state.geo.easting - state.last_geo.easting, 2) + pow(state.geo.northing - state.last_geo.northing, 2));
			const double threshold = 0.2;
			if (dt > threshold) {
				// create fake orientation
				state.yaw_ = atan2(state.geo.northing - state.last_geo.northing, state.geo.easting - state.last_geo.easting);
				state.roll_ = 0;
				state.pitch_ = 0;

				PoseStamped px;
				try {
					px = state.createPose();
				} catch (invalid_gnss_position &e) {
					continue;
				}

				px.timestamp = current_time.toBoost();
				vehicleTrack.push_back(px);
				state.last_geo = state.geo;
				continue;
			}
		}

		double e = 1e-2;
		if (fabs(state.orientation_time_ - state.position_time_) < e) {

			PoseStamped px;
			try {
				px = state.createPose();
			} catch (invalid_gnss_position &e) {
				continue;
			}

			px.timestamp = current_time.toBoost();
			vehicleTrack.push_back(px);
			continue;
		}

	}

	return vehicleTrack;
}



TrajectoryGNSS
TrajectoryGNSS::fromRosBagSatFix(rosbag::Bag &bag, const std::string &topicName, int _plane)
{
	TrajectoryGNSS vehicleTrack;

	RandomAccessBag nmeaBag(bag, topicName);
	if (nmeaBag.messageType() != "sensor_msgs/NavSatFix")
		throw runtime_error("Not GNSS bag");

	PoseStamped prev_pose, pose;
	TQuaternion quat;

	for (int i=0; i<nmeaBag.size(); i++) {
		auto currentMessage = nmeaBag.at<sensor_msgs::NavSatFix>(i);
		ros::Time current_time = nmeaBag.timeAt(i);

		geo_pos_conv geo;

		geo.set_plane(_plane);
		geo.llh_to_xyz(currentMessage->latitude, currentMessage->longitude, currentMessage->altitude);

		tf::Transform pose_transform;
		tf::Quaternion pose_q;

		// pose.header.stamp = ros::Time::now();
		Vector3d position(geo.y(), geo.x(), geo.z());
		double yaw;

		double distance = sqrt(pow(position.y() - prev_pose.position().y(), 2) +
				pow(position.x() - prev_pose.position().x(), 2));
//		std::cout << "distance : " << distance << std::endl;

		if (distance > 0.2)
		{
			yaw = atan2(position.y() - prev_pose.position().y(), position.x() - prev_pose.position().x());
			quat = TQuaternion(0, 0, yaw);
			prev_pose = pose;
		}

		pose = PoseStamped(position, quat, current_time.toBoost());
		vehicleTrack.push_back(pose);
	}

	return vehicleTrack;
}



} /* namespace Vmml */
