/*
 * OxfordDataset.h
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */

#ifndef _OXFORDDATASET_H_
#define _OXFORDDATASET_H_

#include <string>
#include <memory>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "vmml/utilities.h"
#include "vmml/Pose.h"
#include "vmml/Trajectory.h"
#include "vmml/CameraPinholeParams.h"


using Vmml::Pose;
using Vmml::ptime;
using Vmml::tduration;
using Vmml::CameraPinholeParams;

namespace oxf {


struct OxfordRecord {
	ptime timestamp;
	cv::Mat center_image;
	Pose pose;
};


/*
 * XXX: Put files from Oxford SDK in this directory
 */

class OxfordDataset {
public:

	typedef uint64_t timestamp_t;

	OxfordDataset(const std::string &path);
	virtual ~OxfordDataset();

	size_t size() const
	{ return stereoTimestamps.size(); }

	tduration length() const;

	OxfordRecord at(const uint i, bool raw=false) const;

	float hz() const;
	std::vector<uint32_t> desample(const float hz) const;

	Vmml::Trajectory getGroundTruth() const;
	Vmml::Trajectory getImageGroundTruth() const;

	Vmml::CameraPinholeParams getCameraParameters() const
	{ return cameraCenter; }

	std::vector<bool> checkImages() const;

	Vmml::Trajectory getInsTrajectory() const;

	struct GpsPose
	{
		timestamp_t timestamp;
		double
			easting,
			northing,
			altitude,
			latitude,
			longitude;

		double
			velocity_east=0,	// X
			velocity_north=0,	// Y
			velocity_up=0;		// Z

		inline Eigen::Vector3d velocity() const
		{ return Eigen::Vector3d(velocity_east, velocity_north, velocity_up); }
	};

	struct InsPose : public GpsPose
	{
		double
			roll,
			pitch,
			yaw;

		Vmml::PoseStamped toPose() const;
	};

	std::string getId() const
	{ return boost::filesystem::basename(dirpath); }

protected:
	Vmml::Path
		dirpath,	// Directory for target dataset
		pkgpath;	// Directory of our package

	// Need to store exact value of timestamp as read from disk
	std::vector<timestamp_t> stereoTimestamps;

	Vmml::CameraPinholeParams cameraCenter;

	cv::Mat distortionLUT_center_x, distortionLUT_center_y;

	uint getPositionAtDurationSecond(const float &tm) const;

	Vmml::Path imagePathAt(const uint n) const;

	void loadTimestamps();
	void loadModel();
	void loadGps();

	std::vector<GpsPose> gpsPoseTable;
	std::vector<InsPose> insPoseTable;

};

} /* namespace oxf */

#endif /* OXFORD_TEST_OXFORDDATASET_H_ */
