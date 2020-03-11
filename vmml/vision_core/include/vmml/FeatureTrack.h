/*
 * FeatureTrack.h
 *
 *  Created on: Mar 10, 2020
 *      Author: sujiwo
 */

#ifndef VMML_VISION_CORE_INCLUDE_FEATURETRACK_H_
#define VMML_VISION_CORE_INCLUDE_FEATURETRACK_H_


#include <set>
#include <vector>
#include <map>
#include <utility>
#include <opencv2/core.hpp>


namespace Vmml {

/*
 * Track appearances of a feature over a number of frames
 */
class FeatureTrack {
public:

	typedef uint FrameId;

	FeatureTrack() {}

	FeatureTrack(const uint& firstFrameNum, const cv::Point2f &pt)
	{ add(firstFrameNum, pt); }

	void add(const uint& frameNum, const cv::Point2f &pt);

	// for visualization
	std::vector<cv::Point2f> getPoints() const;
	cv::Mat getPointsAsMat() const;

protected:
	std::set<uint> frameNumbers;

	// Map Frame number to feature number at that particular frame
	// (and its position)
	std::map<uint,cv::Point2f> mPoints;

	friend class FeatureTrackList;
};


class FeatureTrackList
{
public:
	typedef FeatureTrack::FrameId FrameId;
	typedef uint TrackId;

	void add (const FeatureTrack& ft);

	inline size_t size() const
	{ return mFeatTracks.size(); }

	uint getNumberOfFeatureTracksAt (const FrameId& fr) const
	{ return frameToFeatureTracks.at(fr).size(); }

	std::vector<const FeatureTrack*> getFeatureTracksAt(const FrameId& frId) const;
//	vector<FeatureTrack*> getFeatureTracksAt(const FrameId& frId);

	cv::Mat createFeatureMask(const cv::Mat& baseMask, const FrameId &f) const;

	cv::Mat trackedFeaturesAtFrame(const FrameId& fr) const;

	const std::set<TrackId>& getFeatureTrackIdsAtFrame(const FrameId &fr) const
	{ return frameToFeatureTracks.at(fr); }

	void addTrackedPoint(const FrameId &fr, const TrackId &tr, const cv::Point2f &pt);

protected:
	std::vector<FeatureTrack> mFeatTracks;
	std::map<FrameId,std::set<TrackId>> frameToFeatureTracks;
};


} /* namespace Vmml */

#endif /* VMML_VISION_CORE_INCLUDE_FEATURETRACK_H_ */
