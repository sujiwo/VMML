/*
 * OxfordDataset.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: sujiwo
 */


#include <fstream>
#include <algorithm>
#include <exception>
#include <cstdio>
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include "csv.h"
#include "datasets/OxfordDataset.h"
#include "utilities.h"

#include <ros/package.h>


using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

#define _OxfordDashboardMask "conf/oxford_mask.png"


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
string OxfordDataset::dSetName = "Oxford";


/*
 * Transformation from INS to stereo camera
 * Might be inaccurate
 */
static const TTransform
baseLinkToOffset = TTransform::from_Pos_Quat(
	Vector3d (0.0, 1.720, 1.070),
	TQuaternion (-0.723, 0.007, 0.002, 0.691));


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


OxfordDataset::OxfordDataset(const OxfordDataset &cp):

	oxfCamera(cp.oxfCamera),
	oxfPath(cp.oxfPath),
	distortionLUT_center_x(cp.distortionLUT_center_x),
	distortionLUT_center_y(cp.distortionLUT_center_y),
	zoomRatio(cp.zoomRatio),
	stereoTimestamps(cp.stereoTimestamps),
	stereoRecords(cp.stereoRecords),
	gpsPoseTable(cp.gpsPoseTable),
	insPoseTable(cp.insPoseTable),
	dashboardMask(cp.dashboardMask)
{
	zoomRatio = cp.zoomRatio;
}


OxfordDataset::OxfordDataset(

	const std::string &dirpath,
	const std::string &modelDir,
	GroundTruthSrc gts) :

	oxfPath (dirpath)
{
	oxfCamera.fx = -1;
	loadTimestamps();

	loadModel(modelDir);

	loadGps();
	loadIns();

	createStereoGroundTruths();

	// try to load mask image
	bfs::path myPath(ros::package::getPath("vmml"));
	auto mask1Path = myPath / _OxfordDashboardMask;
	dashboardMask = cv::imread(mask1Path.string(), cv::IMREAD_GRAYSCALE);

	return;
}


OxfordDataset::Ptr
OxfordDataset::load(const std::string &dirpath,
		const std::string &modelDir,
		GroundTruthSrc gts)
{
	OxfordDataset::Ptr oxDatasetMem(new OxfordDataset(dirpath, modelDir, gts));
	return oxDatasetMem;
}


OxfordDataset::~OxfordDataset()
{
}


tduration
OxfordDataset::getTimeLength()
const
{
	auto t2 = stereoTimestamps.at(stereoTimestamps.size()-1),
		t1 = stereoTimestamps.at(0);
	return boost::posix_time::microseconds(t2-t1);
}


void
OxfordDataset::loadGps()
{
	const string gpsFilePath = oxfPath + "/gps/gps.csv";
	StringTable GPS_s = create_table(gpsFilePath, GpsColumns, true);
	const size_t ss = GPS_s.size();
	gpsPoseTable.resize(ss);
//	gpsTimestamps.resize(ss);

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
}


void
OxfordDataset::loadIns()
{
	const string insFilePath = oxfPath + "/gps/ins.csv";
	StringTable INS_s = create_table(insFilePath, InsColumns, true);
	const size_t ss = INS_s.size();
	insPoseTable.resize(ss);
//	insTimestamps.resize(ss);

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


void OxfordDataset::loadTimestamps()
{
	const string timestampsPath = oxfPath + "/stereo.timestamps";
	StringTable TS = create_table(timestampsPath);
	// It is likely that the last timestamp does not have related image files
	const size_t ss = TS.size()-1;
	stereoTimestamps.resize(ss);

	for (uint32_t i=0; i<ss; i++) {
		const string &tsstr = TS.get(i,0);
		const timestamp_t ts = stoul(tsstr);
		stereoTimestamps[i] = ts;

		OxfordDataItem d(this);
		d.timestamp = ts;
		d.iId = i;
		stereoRecords.insert(make_pair(ts, d));
	}
}


string
OxfordDataItem::getPath(OxfordDataItem::StereoImageT type) const
{
	const string ss = to_string(timestamp);

	switch (type) {
	case StereoLeft:
		return parent->oxfPath + "/stereo/left/" + ss + ".png"; break;
	case StereoCenter:
		return parent->oxfPath + "/stereo/centre/" + ss + ".png"; break;
	case StereoRight:
		return parent->oxfPath + "/stereo/right/" + ss + ".png"; break;
	}
}


ptime
OxfordDataItem::getTimestamp() const
{
	return fromOxfordTimestamp(timestamp);
}


//timestamp_t
//OxfordDataItem::getTimestampLong()
//const
//{ return toOxfordTimestamp(iTimestamp); }


cv::Mat
OxfordDataItem::getImage(StereoImageT t)
const
{
	const string path = getPath(t);
	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);

	// XXX: need better demosaicing algorithms
	cv::cvtColor(img, img, CV_BayerGB2BGR);
	img = parent->undistort(img);

	if (parent->zoomRatio==1.0)
		return img;

	else {
		cv::resize(img, img, cv::Size(), parent->zoomRatio, parent->zoomRatio, cv::INTER_CUBIC);
		return img;
	}
}


Pose fromINS(const InsPose &ps)
{
	return TTransform::from_XYZ_RPY(
		Vector3d(
			ps.easting + OriginCorrectionEasting,
			ps.northing + OriginCorrectionNorthing,
			ps.altitude),
		ps.roll, ps.pitch, ps.yaw
	);
}


void interpolateFromINS (
	uint64_t timestamp,
	const InsPose &ps1,
	const InsPose &ps2,
	Pose &poseResult,
	Vector3d &velocityResult)
{
	assert(timestamp >= ps1.timestamp and timestamp<=ps2.timestamp);

	Pose px1 = fromINS(ps1),
		px2 = fromINS(ps2);

	double ratio = double(timestamp - ps1.timestamp) / double(ps2.timestamp - ps1.timestamp);

	poseResult = TTransform::interpolate(px1, px2, ratio);
	velocityResult = ps1.velocity() + (ps2.velocity()-ps1.velocity())*ratio;
}


void
OxfordDataset::createStereoGroundTruths()
{
	map<uint64_t,InsPose*> tsFinder;

	// 1: create tree for traversing timestamp data
	for (uint32_t i=0; i<insPoseTable.size(); i++) {
		InsPose *p = &insPoseTable.at(i);
		tsFinder.insert(make_pair(insPoseTable[i].timestamp, p));
		if (i>0 and p->timestamp <= (p-1)->timestamp)
			throw range_error("Invalid decreasing timestamp detected");
	}

	// 2
	auto Itx = tsFinder.begin();
	for (uint32_t i=0; i<stereoTimestamps.size(); i++) {

		uint64_t ts = stereoTimestamps[i];
		Pose px;
		Vector3d velocity = Vector3d::Zero();

		if (ts < insPoseTable[0].timestamp) {
			px = fromINS(insPoseTable[0]);
			velocity = insPoseTable[0].velocity();
		}

		else if (ts > insPoseTable[insPoseTable.size()-1].timestamp) {
			px = fromINS(insPoseTable[insPoseTable.size()-1]);
			velocity = insPoseTable[insPoseTable.size()-1].velocity();
		}

		else {
			decltype(Itx) Itx_prev;
			do {
				Itx_prev = Itx;
				++Itx;
			} while (ts > (*Itx).first and Itx!=tsFinder.end());

			const uint64_t ts1 = (*Itx_prev).first,
				ts2 = (*Itx).first;
			const InsPose& ps1 = *tsFinder[ts1],
				&ps2 = *tsFinder[ts2];

			interpolateFromINS(ts, ps1, ps2, px, velocity);
		}

		// Transform INS/baselink position to camera
		px = px * baseLinkToOffset;

		stereoRecords.at(ts).groundTruth = px;
		stereoRecords.at(ts).velocity = velocity;
	}
}


const OxfordDataItem&
OxfordDataset::at(dataItemId i) const
{
	timestamp_t ts = stereoTimestamps.at(i);
	return stereoRecords.at(ts);
}


GenericDataItem::ConstPtr
OxfordDataset::get(dataItemId i) const
{
	timestamp_t ts = stereoTimestamps.at(i);
	OxfordDataItem::ConstPtr oxfItem(&stereoRecords.at(ts),
		// do not delete this pointer
		[](OxfordDataItem const* p){}
	);
	return oxfItem;
}


void
OxfordDataset::loadModel(const string &modelDir)
{
	string
		centerLut = modelDir + "/stereo_narrow_left_distortion_lut.bin",
		centerIntrinsic = modelDir + "/stereo_narrow_left.txt";

	// LUT distortion correction table
	std::ifstream lutfd (centerLut, ifstream::ate|ifstream::binary);
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
	StringTable intr = create_table(centerIntrinsic);
	oxfCamera.fx = stod(intr.get(0,0));
	oxfCamera.fy = stod(intr.get(0,1));
	oxfCamera.cx = stod(intr.get(0,2));
	oxfCamera.cy = stod(intr.get(0,3));

	auto &d0 = this->at(0);
	cv::Mat img0 = cv::imread(d0.getPath());
	oxfCamera.width = img0.cols;
	oxfCamera.height = img0.rows;

	if (oxfCamera.width * oxfCamera.height != distortionLUT_center_x.cols)
		throw runtime_error("Mismatched image size and model size");
	distortionLUT_center_x = distortionLUT_center_x.reshape(0, oxfCamera.height);
	distortionLUT_center_y = distortionLUT_center_y.reshape(0, oxfCamera.height);

}


cv::Mat
OxfordDataset::undistort (cv::Mat &src)
{
	cv::Mat target;
	cv::remap(src, target, distortionLUT_center_x, distortionLUT_center_y, cv::INTER_LINEAR);
	return target;
}


cv::Mat
OxfordDataset::getMask()
{
	cv::Mat imgrs;
	if (zoomRatio==1.0)
		return dashboardMask.clone();
	else {
		cv::resize(dashboardMask, imgrs, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
		return imgrs;
	}
}


OxfordDataset::Ptr
OxfordDataset::timeSubset (double startTimeOffsetSecond, double durationSecond)
const
{
	OxfordDataset *mycopy = new OxfordDataset(*this);

	// Determine start time in second
	double absStartTimeSecond = double(stereoTimestamps[0]/1e6) + startTimeOffsetSecond;
	if (durationSecond<0) {
		double lastTime = double(stereoTimestamps.back()) / 1e6;
		durationSecond = lastTime - absStartTimeSecond;
	}

	// Create subset
	uint32_t sId = 0;
	mycopy->stereoTimestamps.clear();
	mycopy->stereoRecords.clear();
	mycopy->gpsPoseTable.clear();
	mycopy->insPoseTable.clear();

	for (auto it = stereoTimestamps.begin(); it!=stereoTimestamps.end(); ++it) {
		timestamp_t curTimestamp = *it;

		double ts = double(*it) / 1e6;
		if (ts>=absStartTimeSecond) {
			if (ts > absStartTimeSecond + durationSecond)
				break;

			mycopy->stereoTimestamps.push_back(curTimestamp);

			OxfordDataItem d(mycopy);
			d.timestamp = curTimestamp;
			d.iId = sId;
			d.groundTruth = this->stereoRecords.at(curTimestamp).groundTruth;
			mycopy->stereoRecords.insert(make_pair(curTimestamp, d));

			sId++;
		}
	}

	for (auto &gp: gpsPoseTable) {
		double gpTs = double(gp.timestamp) / 1e6;
		if (absStartTimeSecond < gpTs and gpTs<absStartTimeSecond + durationSecond) {
			mycopy->gpsPoseTable.push_back(gp);
		}
	}

	for (auto &ipp: insPoseTable) {
		double ipTs = double(ipp.timestamp) / 1e6;
		if (absStartTimeSecond < ipTs and ipTs<absStartTimeSecond + durationSecond) {
			mycopy->insPoseTable.push_back(ipp);
		}
	}

	return OxfordDataset::Ptr(mycopy);
}


dataItemId
OxfordDataset::getLowerBound (const ptime &t) const
{
	auto it = std::lower_bound(stereoTimestamps.begin(), stereoTimestamps.end(), toOxfordTimestamp(t));
	return (dataItemId)(it-stereoTimestamps.begin());
}


void
OxfordDataset::setZoomRatio(float r)
{
	zoomRatio = r;
	oxfCamera = oxfCamera * r;
}


GenericDataItem::ConstPtr
OxfordDataset::atDurationSecond (const double second) const
{
	timestamp_t tx=stereoTimestamps[0] + static_cast<timestamp_t>(second*1e6);
	return atApproximate(tx);
}


OxfordDataItem::ConstPtr
OxfordDataset::atApproximate(timestamp_t t) const
{
	auto it = std::lower_bound(stereoTimestamps.begin(), stereoTimestamps.end(), t);
	return atTime(*it);
}


Trajectory
OxfordDataset::getCameraTrajectory(const ptime timeStart, const ptime timeStop) const
{
	timestamp_t
		t1 = (timeStart==MIN_TIME ? stereoTimestamps.front() : toOxfordTimestamp(timeStart)),
		t2 = (timeStop ==MAX_TIME ? stereoTimestamps.back()  : toOxfordTimestamp(timeStop) );

	Trajectory egoTrack;

	for (uint i=0; i<stereoTimestamps.size(); ++i) {
		auto ts = stereoTimestamps[i];
		if (t1 <= ts and ts <= t2) {
			auto px = stereoRecords.at(ts);
			ptime t = fromSeconds(double(ts)/1e6);
			PoseStamped ego(px.getPosition(), px.getOrientation(), t);
			egoTrack.push_back(ego);
		}
	}

	return egoTrack;
}


OxfordImagePreprocessor::OxfordImagePreprocessor (const string &modelDir)
{
	string
		centerLut = modelDir + "/stereo_narrow_left_distortion_lut.bin",
		centerIntrinsic = modelDir + "/stereo_narrow_left.txt";

	// LUT distortion correction table
	std::ifstream lutfd (centerLut, ifstream::ate|ifstream::binary);
	const size_t lutfdsize = lutfd.tellg();
	if (lutfdsize%sizeof(double) != 0)
		throw runtime_error("File size is not correct");
	lutfd.seekg(ifstream::beg);

	cv::Mat distortionLUT_center (2, lutfdsize/(sizeof(double)*2), CV_64F);
	lutfd.read((char*)distortionLUT_center.ptr(0), lutfdsize/2);
	lutfd.read((char*)distortionLUT_center.ptr(1), lutfdsize/2);
	distortionLUT_center.row(0).convertTo(distortionLUT_center_x, CV_32F);
	distortionLUT_center.row(1).convertTo(distortionLUT_center_y, CV_32F);

	distortionLUT_center_x = distortionLUT_center_x.reshape(0, 960);
	distortionLUT_center_y = distortionLUT_center_y.reshape(0, 960);
}


cv::Mat
OxfordImagePreprocessor::process(const cv::Mat &rawImage) const
{
	assert(rawImage.channels()==1);
	cv::Mat colorImage;
	// XXX: need better demosaicing algorithms
	cv::cvtColor(rawImage, colorImage, CV_BayerGB2RGB);

	cv::remap(colorImage, colorImage, distortionLUT_center_x, distortionLUT_center_y, cv::INTER_LINEAR);

	if (zoomRatio==1.0)
		return colorImage;
	else {
		cv::resize(colorImage, colorImage, cv::Size(), zoomRatio, zoomRatio, cv::INTER_CUBIC);
		return colorImage;
	}
}


cv::Mat
OxfordImagePreprocessor::load (const string &rawImagePath)
const
{
	cv::Mat rawImg = cv::imread(rawImagePath, cv::IMREAD_GRAYSCALE);
	return process (rawImg);
}


