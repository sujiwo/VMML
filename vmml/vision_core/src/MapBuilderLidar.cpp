/*
 * MapBuilderLidar.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: sujiwo
 */

#include "MapBuilderLidar.h"
#include "Matcher.h"
#include "Triangulation.h"


using namespace std;
using pcl::PointXYZ;


namespace Vmml {


MapBuilderLidar::MapBuilderLidar(const CameraPinholeParams &camera0, const std::string &mapVocabularyPath) :
	MapBuilder(camera0, mapVocabularyPath)
{
	// TODO Auto-generated constructor stub

}


MapBuilderLidar::~MapBuilderLidar()
{
	// TODO Auto-generated destructor stub
}


MapBuilderLidar::LidarImageFrame::LidarImageFrame(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &_scan, std::shared_ptr<VisionMap> &_parent, ptime _lidarTs) :
	TmpFrame(img, _parent),
	lidarScan(_scan),
	lidarTs(_lidarTs)
{}


void
MapBuilderLidar::LidarImageFrame::setImage(cv::Mat img)
{
	image = img;
	computeFeatures(parent->getFeatureDetector());
}


MapBuilderLidar::LidarImageFrame::Ptr
MapBuilderLidar::LidarImageFrame::create(cv::Mat img, LocalLidarMapper::CloudType::ConstPtr &_scan, std::shared_ptr<VisionMap> &_parent, ptime _lidarTs)
{ return Ptr(new LidarImageFrame(img, _scan, _parent, _lidarTs)); }


void
MapBuilderLidar::run(
	const rosbag::Bag &bagFd,
	const std::string &velodyneCalibrationFilePath,
	const std::string &velodyneTopic,
	const std::string &imageTopic,
	const float imageScale)
{
	imageSource.reset(new ImageBag(bagFd, imageTopic, imageScale));
	velScanSource.reset(new LidarScanBag(bagFd, velodyneTopic, velodyneCalibrationFilePath));

	const int N = velScanSource->size();
	for (int ild=0; ild<N; ++ild) {

		// Fetch lidar scan
		ptime lidarTs;
		auto lidarScan = velScanSource->getUnfiltered<PointXYZ>(ild, &lidarTs);
		// Delay fetching images
		currentFrame = LidarImageFrame::create(cv::Mat(), lidarScan, vMap, lidarTs);

		// Run NDT
		lidarTracker.feed(lidarScan, lidarTs, currentFrame->frLog);

		// first frame ?
		if (lastAnchor==0) {
			ptime imageTs;
			currentFrame->setImage(getImage(lidarTs, imageTs));
			currentFrame->timestamp = imageTs;
			setPoseFromLidar(Pose::Identity(), *currentFrame);

			auto K1 = KeyFrame::fromBaseFrame(*currentFrame, vMap, 0, imageTs);
			vMap->addKeyFrame(K1);
			lastAnchor = K1->getId();
			callFrameFunction();
		}

		else {
			if (currentFrame->frLog.hasScanFrame==true) {
				ptime imageTs;
				currentFrame->setImage(getImage(lidarTs, imageTs));
				currentFrame->timestamp = imageTs;
				track();
			}
		}

	}
}


cv::Mat
MapBuilderLidar::getImage(const ptime &ts, ptime &imageTs)
{
	imageFrameNumber = imageSource->getPositionAtTime(ros::Time::fromBoost(ts));
	imageTs = imageSource->timeAt(imageFrameNumber).toBoost();
	return imageSource->at(imageFrameNumber);
}


/*
 * This function performs image-side of tracking
 */
bool
MapBuilderLidar::track()
{
	auto Knext = KeyFrame::fromBaseFrame(*currentFrame, vMap, 0, currentFrame->timestamp);
	auto Kanchor = vMap->keyframe(lastAnchor);
	vMap->addKeyFrame(Knext);

	// Get pose in metric
	const auto &prevFrameLog = lidarTracker.getScanLog(currentFrame->frLog.prevScanFrame);
	TTransform metricMove = prevFrameLog.poseAtScan.inverse() * currentFrame->frLog.poseAtScan;
	Pose currentFramePose = Kanchor->pose() * metricMove;

	// Image matching
	Matcher::PairList vFeatPairs1;
	int Ns1 = Matcher::matchBruteForce(*Kanchor, *currentFrame, vFeatPairs1);

	// Find transformation
	Matcher::PairList vFeatPairs2;
	TTransform motion = Matcher::calculateMovement(*Kanchor, *currentFrame, vFeatPairs1, vFeatPairs2);
	if (vFeatPairs2.size()<10)
		return false;

	// Scale translation
	Vector3d t = motion.translation();
	t = t * double(metricMove.translation().norm());
	motion = TTransform::from_Pos_Quat(t, motion.orientation());
	Pose newFramePose = Kanchor->pose() * motion;
	setPoseFromLidar(newFramePose, *Knext);
	TTransform realMotion = Kanchor->pose().inverse() * Knext->pose();

	// Build point cloud from image triangulation
	map<uint, Vector3d> mapPoints;
	float parallax;
	TriangulateCV(*Kanchor, *Knext, vFeatPairs2, mapPoints, &parallax);

	// Call NDT for 2nd time
	lidarTracker.matching2nd(currentFrame->lidarScan, realMotion);

	// Build visibility graph

	lastAnchor = Knext->getId();
	return true;
}


void
MapBuilderLidar::setPoseFromLidar(const Pose &p, BaseFrame &f)
{
	f.setPose(p * lidarToCamera);
}

} /* namespace Vmml */
