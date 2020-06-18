/*
 * OxfordDataset.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */

#include <algorithm>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <ros/package.h>
#include "csv.h"
#include "OxfordDataset.h"


using namespace std;
using Vmml::PoseStamped;


namespace oxf {


/*
 * XXX: Oxford Timestamp is in Microsecond
 */

ptime fromOxfordTimestamp (const uint64_t ot)
{
	static const ptime epoch(boost::gregorian::date(1970, 1, 1));
	return epoch + boost::posix_time::microseconds(ot);
}


uint64_t toOxfordTimestamp (const ptime &t)
{
	static const ptime epoch(boost::gregorian::date(1970, 1, 1));
	auto d = t-epoch;
	return d.total_microseconds();
}


OxfordDataset::OxfordDataset(const std::string &path) :
	dirpath(path),
	pkgpath(ros::package::getPath("oxford_test"))
{
	loadTimestamps();
	loadModel();
	loadGps();
}


OxfordDataset::~OxfordDataset() {
	// TODO Auto-generated destructor stub
}


void
OxfordDataset::loadTimestamps()
{
	const string timestampsPath = (dirpath / "stereo.timestamps").string();
	StringTable TS = create_table(timestampsPath);
	// It is likely that the last timestamp does not have related image files
	const size_t ss = TS.size()-1;
	stereoTimestamps.resize(ss);

	for (uint32_t i=0; i<ss; i++) {
		const string &tsstr = TS.get(i,0);
		const uint64 ts = stoul(tsstr);
		stereoTimestamps[i] = ts;
	}
}


Vmml::Path
OxfordDataset::imagePathAt(const uint n) const
{
	return dirpath
		/ "stereo/centre"
		/ (to_string(stereoTimestamps.at(n))+".png");
}


static const set<int> GpsColumns ({0,9,8,4,2,3});
/*
 * These are the columns of INS pose table that are currently used:
 * - timestamp
 * - easting
 * - northing
 * - altitude
 * - roll
 * - pitch
 * - yaw
 * - velocity_east
 * - velocity_north
 * - velocity_down
 * - latitude
 * - longitude
 */
static const set<int> InsColumns ({0,6,5,4,12,13,14,10,9,11,2,3});

void
OxfordDataset::loadGps()
{
	const auto gpsFilePath = dirpath / "gps/gps.csv";
	StringTable GPS_s = create_table(gpsFilePath.string(), GpsColumns, true);
	size_t ss = GPS_s.size();
	gpsPoseTable.resize(ss);

	for (uint i=0; i<ss; i++) {
		GpsPose ps;
			ps.timestamp = stoul(GPS_s.get(i, "timestamp"));
			ps.easting = stod(GPS_s.get(i, "easting"));
			ps.northing = stod(GPS_s.get(i, "northing"));
			ps.altitude = stod(GPS_s.get(i, "altitude"));
			ps.latitude = stod(GPS_s.get(i, "latitude"));
			ps.longitude = stod(GPS_s.get(i, "longitude"));
		gpsPoseTable[i] = ps;
	}

	const auto insFilePath = dirpath / "gps/ins.csv";
	StringTable INS_s = create_table(insFilePath.string(), InsColumns, true);
	ss = INS_s.size();
	insPoseTable.resize(ss);

	for (uint i=0; i<ss; i++) {
		InsPose is;
			is.timestamp = stoul(INS_s.get(i, "timestamp"));
			is.easting = stod(INS_s.get(i, "easting"));
			is.northing = stod(INS_s.get(i, "northing"));
			is.altitude = stod(INS_s.get(i, "altitude"));
			is.latitude = stod(INS_s.get(i, "latitude"));
			is.longitude = stod(INS_s.get(i, "longitude"));
			is.roll = stod(INS_s.get(i, "roll"));

			// Correct pitch & yaw rotations to more sensible ROS convention;
			// otherwise you will have problems later
			is.pitch = -stod(INS_s.get(i, "pitch"));
			is.yaw = -stod(INS_s.get(i, "yaw"));

			// Velocity
			is.velocity_north = stod(INS_s.get(i, "velocity_north"));
			is.velocity_east = stod(INS_s.get(i, "velocity_east"));
			is.velocity_up = -stod(INS_s.get(i, "velocity_down"));

		insPoseTable[i] = is;
	}
}


Vmml::PoseStamped
OxfordDataset::InsPose::toPose() const
{
	Eigen::Vector3d pos(easting, northing, altitude);
	auto q = Vmml::fromRPY(roll, pitch, yaw);
	return PoseStamped(pos, q, fromOxfordTimestamp(timestamp));
}


Vmml::Trajectory
OxfordDataset::getInsTrajectory() const
{
	Vmml::Trajectory gpsTrack;

	for (uint i=0; i<insPoseTable.size(); ++i) {
		auto &psGps = insPoseTable.at(i);
		PoseStamped pxGps = psGps.toPose();
		gpsTrack.push_back(pxGps);
	}

	return gpsTrack;
}


OxfordRecord
OxfordDataset::at(const uint i, bool raw) const
{
	OxfordRecord recz;
	recz.timestamp = fromOxfordTimestamp(stereoTimestamps.at(i));

	auto imgPath = imagePathAt(i);
	recz.center_image = cv::imread(
		imgPath.string(),
		cv::IMREAD_GRAYSCALE);

	// Skip treatment for invalid images
	if (raw==false and recz.center_image.empty()==false) {
		cv::cvtColor(recz.center_image, recz.center_image, CV_BayerGB2RGB);
		cv::remap(recz.center_image, recz.center_image, distortionLUT_center_x, distortionLUT_center_y, cv::INTER_LINEAR);
	}

	return recz;
}


std::vector<bool>
OxfordDataset::checkImages() const
{
	vector<bool> hasImages(stereoTimestamps.size(), true);

	for (uint i=0; i<size(); ++i) {
		auto imgPath = imagePathAt(i);
		hasImages[i] = boost::filesystem::exists(imgPath);
//		auto img = cv::imread(imgPath.string(), cv::IMREAD_GRAYSCALE);
//		hasImages[i] = !img.empty();
	}

	return hasImages;
}


tduration
OxfordDataset::length() const
{
	return fromOxfordTimestamp(stereoTimestamps.back()) -
		fromOxfordTimestamp(stereoTimestamps.front());
}


void
OxfordDataset::loadModel()
{
	// XXX: Find path to these files

	const Vmml::Path
		centerLut = pkgpath/"model"/"stereo_narrow_left_distortion_lut.bin",
		centerIntrinsic = pkgpath/"model"/"stereo_narrow_left.txt";

	// LUT distortion correction table
	std::ifstream lutfd (centerLut.string(), ifstream::ate|ifstream::binary);
	const size_t lutfdsize = lutfd.tellg();
	if (lutfdsize%sizeof(double) != 0)
		throw runtime_error("File size is not correct");
	lutfd.seekg(ifstream::beg);

	cv::Mat distortionLUT_center (2, lutfdsize/(sizeof(double)*2), CV_64F);
	lutfd.read((char*)distortionLUT_center.ptr(0), lutfdsize/2);
	lutfd.read((char*)distortionLUT_center.ptr(1), lutfdsize/2);
	distortionLUT_center.row(0).convertTo(distortionLUT_center_x, CV_32F);
	distortionLUT_center.row(1).convertTo(distortionLUT_center_y, CV_32F);

	// Camera intrinsic parameters
	StringTable intr = create_table(centerIntrinsic.string());
	cameraCenter.fx = stod(intr.get(0,0));
	cameraCenter.fy = stod(intr.get(0,1));
	cameraCenter.cx = stod(intr.get(0,2));
	cameraCenter.cy = stod(intr.get(0,3));

	auto d0 = at(0, true);
	cameraCenter.width = d0.center_image.cols;
	cameraCenter.height = d0.center_image.rows;

	if (cameraCenter.width * cameraCenter.height != distortionLUT_center_x.cols)
		throw runtime_error("Mismatched image size and model size");
	distortionLUT_center_x = distortionLUT_center_x.reshape(0, cameraCenter.height);
	distortionLUT_center_y = distortionLUT_center_y.reshape(0, cameraCenter.height);
}


float
OxfordDataset::hz() const
{
	return float(size()) / Vmml::toSeconds(length());
}


std::vector<uint32_t>
OxfordDataset::desample(const float hz, double offsetStart, double offsetStop) const
{
	if (offsetStop<0)
		offsetStop = Vmml::toSeconds(length());
	assert(offsetStart <= offsetStop);

	vector<uint32_t> resampled;

	const double lengthInSeconds = Vmml::toSeconds(length());

	uint posWk = 0, nextWk;
	const double tIntrv = 1.0 / hz;
	for (double twork=offsetStart; twork<offsetStop; twork+=1.0) {
		double tMax = min(twork+1.0, lengthInSeconds);
		double tm = twork+tIntrv;
		while (tm < tMax) {
			uint p = getPositionAtDurationSecond(tm);
			resampled.push_back(p);
			tm += tIntrv;
		}
	}

	return resampled;
}


uint
OxfordDataset::getPositionAtDurationSecond(const float &tm) const
{
	auto td = Vmml::durationFromSeconds(tm);
	auto ctime = fromOxfordTimestamp(stereoTimestamps.front()) + td;
	assert (ctime <= fromOxfordTimestamp(stereoTimestamps.back()));

	timestamp_t tx = toOxfordTimestamp(ctime);
	auto it = std::lower_bound(stereoTimestamps.begin(), stereoTimestamps.end(), tx);

	return it-stereoTimestamps.begin();
}


} /* namespace oxf */


