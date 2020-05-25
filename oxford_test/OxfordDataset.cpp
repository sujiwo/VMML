/*
 * OxfordDataset.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */

#include <fstream>
#include <opencv2/highgui.hpp>
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
	dirpath(path)
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

	if (raw==true)
		recz.center_image = cv::imread(
			(dirpath
				/ "stereo/centre"
				/ to_string(stereoTimestamps.at(i))).string(),
			cv::IMREAD_GRAYSCALE);
	else {
		// XXX: unfinished
	}

	return recz;
}


void
OxfordDataset::loadModel()
{
	// XXX: Find path to these files

	const string
		centerLut = "/stereo_narrow_left_distortion_lut.bin",
		centerIntrinsic = "/stereo_narrow_left.txt";

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
	cameraCenter.fx = stod(intr.get(0,0));
	cameraCenter.fy = stod(intr.get(0,1));
	cameraCenter.cx = stod(intr.get(0,2));
	cameraCenter.cy = stod(intr.get(0,3));

	auto &d0 = at(0, true);
	cameraCenter.width = d0.center_image.cols;
	cameraCenter.height = d0.center_image.rows;

	if (cameraCenter.width * cameraCenter.height != distortionLUT_center_x.cols)
		throw runtime_error("Mismatched image size and model size");
	distortionLUT_center_x = distortionLUT_center_x.reshape(0, cameraCenter.height);
	distortionLUT_center_y = distortionLUT_center_y.reshape(0, cameraCenter.height);
}

} /* namespace oxf */


