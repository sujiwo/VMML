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


using Vmml::Pose;
using Vmml::ptime;
using Vmml::tduration;

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
	OxfordDataset(const std::string &path);
	virtual ~OxfordDataset();

	size_t size() const
	{ return stereoTimestamps.size(); }

	tduration length() const;

	OxfordRecord at(const uint i) const;

	Vmml::Trajectory getGroundTruth() const;
	Vmml::Trajectory getImageGroundTruth() const;

protected:
	Vmml::Path dirpath;

	std::vector<ptime> stereoTimestamps;

private:
	void loadTimestamps();
	void loadModel();
};

} /* namespace oxf */

#endif /* OXFORD_TEST_OXFORDDATASET_H_ */
