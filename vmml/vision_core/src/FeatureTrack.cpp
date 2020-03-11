/*
 * FeatureTrack.cpp
 *
 *  Created on: Mar 10, 2020
 *      Author: sujiwo
 */

#include <exception>
#include <opencv2/imgproc.hpp>
#include "vmml/FeatureTrack.h"

using namespace std;


namespace Vmml {


void
FeatureTrack::add(const uint& frameNum, const cv::Point2f &pt)
{
	frameNumbers.insert(frameNum);
	mPoints[frameNum] = pt;
}


vector<cv::Point2f>
FeatureTrack::getPoints() const
{
	vector<cv::Point2f> points;

	for (auto &pr: frameNumbers) {
		points.push_back(mPoints.at(pr));
	}

	return points;
}


cv::Mat
FeatureTrack::getPointsAsMat() const
{
	auto vpt = getPoints();
	cv::Mat ptMat(vpt.size(), 2, CV_32S);
	for (uint r=0; r<vpt.size(); ++r) {
		ptMat.at<int>(r,0) = int(vpt[r].x);
		ptMat.at<int>(r,1) = int(vpt[r].y);
	}

	return ptMat;
}


void
FeatureTrackList::add (const FeatureTrack& ft)
{
	mFeatTracks.push_back(ft);
	TrackId currentTrackId=mFeatTracks.size()-1;

	for (auto &f: ft.frameNumbers) {
		if (frameToFeatureTracks.find(f)==frameToFeatureTracks.end()) {
			frameToFeatureTracks.insert(make_pair(f, set<TrackId>({currentTrackId})));
		}
		else {
			frameToFeatureTracks.at(f).insert(currentTrackId);
		}
	}
}


vector<const FeatureTrack*>
FeatureTrackList::getFeatureTracksAt(const FrameId& frId) const
{
	vector<const FeatureTrack*> rft;

	for (auto &tId: frameToFeatureTracks.at(frId)) {
		rft.push_back(&mFeatTracks.at(tId));
	}

	return rft;
}


cv::Mat
FeatureTrackList::trackedFeaturesAtFrame(const FrameId& fr) const
{
	auto &tracks = frameToFeatureTracks.at(fr);
	cv::Mat rtrackMat(tracks.size(), 2, CV_32F);

	uint i=0;
	for(auto t: tracks) {
		auto &track = mFeatTracks.at(t);
		auto &point = track.mPoints.at(fr);
		rtrackMat.at<float>(i,0) = point.x;
		rtrackMat.at<float>(i,1) = point.y;
		++i;
	}

	return rtrackMat;
}


cv::Mat
FeatureTrackList::createFeatureMask(const cv::Mat& baseMask, const FrameId &f)
const
{
	assert(baseMask.empty()==false);
	auto mask = baseMask.clone();

	for (auto &_tr: getFeatureTracksAt(f)) {
		auto &tr = *_tr;
		for (auto pt: tr.getPoints()) {
			cv::circle(mask, pt, 5, cv::Scalar(0), -1);
		}
	}

	return mask;
}


void
FeatureTrackList::addTrackedPoint(const FrameId &fr, const TrackId &tr, const cv::Point2f &pt)
{
	mFeatTracks.at(tr).add(fr, pt);
	if (frameToFeatureTracks.find(fr)==frameToFeatureTracks.end()) {
		frameToFeatureTracks.insert(make_pair(fr, set<TrackId>({tr})));
	}
	else {
		frameToFeatureTracks.at(fr).insert(tr);
	}
}


void
FeatureTrackList::cleanup()
{
	ftLock.lock();

	set<TrackId> toBeRemoved;
	for (TrackId i=0; i<mFeatTracks.size(); ++i) {
		if (mFeatTracks[i].frameNumbers.size() < minTrackSize)
			toBeRemoved.insert(i);
	}

	for (auto &pr: frameToFeatureTracks) {
//		if (pr.second.)
		for (auto &t: toBeRemoved) {
			if (pr.second.find(t)!=pr.second.end())
				pr.second.erase(t);
		}
	}

	// XXX: unfinished
	ftLock.unlock();
}

} /* namespace Vmml */
