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


OxfordRecord
OxfordDataset::at(const uint i, bool raw) const
{
	OxfordRecord recz;
	recz.timestamp = fromOxfordTimestamp(stereoTimestamps.at(i));

	auto imgPath = dirpath
			/ "stereo/centre"
			/ (to_string(stereoTimestamps.at(i))+".png");
	recz.center_image = cv::imread(
		imgPath.string(),
		cv::IMREAD_GRAYSCALE);

	if (raw==false) {
		cv::cvtColor(recz.center_image, recz.center_image, CV_BayerGB2BGR);
		cv::remap(recz.center_image, recz.center_image, distortionLUT_center_x, distortionLUT_center_y, cv::INTER_LINEAR);
	}

	return recz;
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
OxfordDataset::desample(const float hz) const
{
	vector<uint32_t> resampled;

	const double lengthInSeconds = Vmml::toSeconds(length());

	uint posWk = 0, nextWk;
	const double tIntrv = 1.0 / hz;
	for (double twork=0.0; twork<lengthInSeconds; twork+=1.0) {
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


