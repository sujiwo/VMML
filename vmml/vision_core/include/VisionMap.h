/*
 * VisionMap.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_VISIONMAP_H_
#define VMML_CORE_VISIONMAP_H_

#include <vector>
#include <string>
#include <memory>
#include <set>
#include <map>
#include <Eigen/Eigen>
#include <limits>
#include <mutex>

#include <boost/serialization/serialization.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/graph/adjacency_list.hpp>

#include "utilities.h"
#include "CameraPinholeParams.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Trajectory.h"
#include "ORBVocabulary.h"


namespace Vmml {


class VisionMap
{
public:
	VisionMap();
	virtual ~VisionMap();

	void reset();

	int addCameraParameter (const CameraPinholeParams &vscamIntr);

	bool loadVocabulary(const std::string &vocabPath);

	inline const CameraPinholeParams& getCameraParameter(int cameraId) const
	{ return cameraList[cameraId]; }

	int getNumberOfCameras () const
	{ return cameraList.size(); }

	bool addKeyFrame(KeyFrame::Ptr frame);
	bool addMapPoint(MapPoint::Ptr mpPoint);
	void addMapPointVisibility(const mpid &mp, const kfid &kf, const kpid &kp);

	inline KeyFrame::Ptr getKeyFrameById (const kfid &i) const
	{ return keyframeInvIdx.at(i); }

	inline MapPoint::Ptr getMapPointById (const mpid &i) const
	{ return mappointInvIdx.at(i); }

	std::vector<kfid> getKeyFrameList() const;
	std::vector<mpid> getMapPointList() const;

	inline KeyFrame::Ptr keyframe(const kfid &i) const
	{ return keyframeInvIdx.at(i); }

	inline MapPoint::Ptr mappoint (const mpid &i) const
	{ return mappointInvIdx.at(i); }

	std::vector<kfid> allKeyFrames () const;
	std::vector<mpid> allMapPoints () const;

	int numOfKeyFrames() const
	{ return keyframeInvIdx.size(); }

	int numOfMapPoints() const
	{ return mappointInvIdx.size(); }

	std::vector<mpid> getVisibleMapPoints (const kfid &kf) const;

	inline std::map<kpid,mpid> getAllMapPointProjectionsAt (const kfid &kf) const
	{ return framePointsInv.at(kf); }

	inline int countRelatedKeyFrames(const mpid &i) const
	{ return pointAppearances.at(i).size(); }

	inline std::set<kfid> getRelatedKeyFrames (const mpid &i) const
	{ return pointAppearances.at(i); }

	inline kpid getKeyPointId (const kfid k, const mpid p) const
	{ return framePoints.at(k).at(p); }

	inline mpid getMapPointByKeypoint (const kfid k, const kpid p)
	{ return framePointsInv.at(k).at(p); }

	// XXX: Causes SIGSEGV when framePoints are empty
	std::map<mpid,kpid> allMapPointsAtKeyFrame(const kfid f) const;

	inline uint getNumberObservations(const mpid &mp) const
	{ return pointAppearances.at(mp).size(); }

	uint getTrackedMapPointsAt(const kfid &kf, const uint minNumberOfObservation=0) const;

	// Map building
/*
	bool estimateStructure (const kfid &keyFrame1, const kfid &keyFrame2, double translationHint=-1.0);

	bool estimateAndTrack (const kfid &kfid1, const kfid &kfid2, const double metricDisposition=1.0);
*/

	void trackMapPoints (const kfid kf1, const kfid kf2);

	bool removeMapPoint (const mpid &i);
	bool removeMapPointsBatch (const vector<mpid> &mplist);

	// Writing & Reading a map in disk
	bool save (const std::string &path) const;
	bool load (const std::string &path);

	/*
	 * Map debugging routines
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr dumpPointCloudFromMapPoints () const;

	std::vector<std::pair<Eigen::Vector3d,Eigen::Quaterniond> >
		dumpCameraPoses () const;

	void dumpCameraPoses (Trajectory &track) const;

	/*
	 * Information about this map
	 */
	typedef std::map<std::string,std::string> mapKeyValueInfo;

	inline void setInfo (const std::string &key, const std::string &value)
	{ keyValueInfo[key] = value; };

	inline const std::string getInfo (const std::string &key) const
	{ return keyValueInfo.at(key); }

	inline const mapKeyValueInfo& getAllInfo() const
	{ return keyValueInfo; }

	/*
	 * KeyFrame Graph routines
	 */
	std::vector<kfid> getOrderedRelatedKeyFramesFrom (const kfid k, int howMany=-1) const;

	std::vector<kfid> getKeyFramesComeInto (const kfid kTarget) const;

	/*
	 * 2D Image Stuff
	 */
	const cv::Ptr<cv::FeatureDetector>& getFeatureDetector () const
	{ return featureDetector; }
	const cv::Ptr<cv::DescriptorMatcher>& getDescriptorMatcher () const
	{ return descriptorMatcher; }

	/*
	 * Place Recognition:
	 * Find matched keyframe candidates from database using BoW method.
	 * The candidates are sorted according to number of similar words
	 */
	std::vector<kfid>
	findCandidates (const BaseFrame &f) const;

	void updateCovisibilityGraph(const kfid k);

	void updateMapPointDescriptor(const mpid mp);

	typedef std::shared_ptr<VisionMap> Ptr;

protected:

	friend class KeyFrame;
	friend class MapPoint;

	std::shared_ptr<std::mutex> keyframeInvIdx_mtx;
	std::map<kfid, KeyFrame::Ptr> keyframeInvIdx;

	std::shared_ptr<std::mutex> mappointInvIdx_mtx;
	std::map<mpid, MapPoint::Ptr> mappointInvIdx;

	// Relationship between MapPoint and KeyFrames

	// Maps MapPoint to set of KeyFrames in which it appears
	std::map<mpid, std::set<kfid> > pointAppearances;

	// List all map points that appears in a keyframe with
	// their ID of KeyPoint (ie. their projections)
	// KeyFrame -> (MapPoint, KeyPoint)
	std::map<kfid,
		std::map<mpid, kpid> > framePoints;

	// List of all map points with inverted of the above
	// It will be useful eg. to check projection (see Localizer.cpp)
	// KeyFrame -> (KeyPoint, MapPoint)
	std::map<kfid,
		std::map<kpid, mpid> > framePointsInv;

	// 2D image stuff
	cv::Ptr<cv::FeatureDetector> featureDetector;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

	std::vector<CameraPinholeParams> cameraList;

	/*
	 * KeyFrame Graph
	 */
	typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
	typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, boost::no_property, EdgeWeightProperty> KeyFrameGraph;
	typedef boost::graph_traits<KeyFrameGraph>::edge_iterator edge_iterator;

	KeyFrameGraph covisibility;
	std::map<kfid,KeyFrameGraph::vertex_descriptor> kfVtxMap;
	std::map<KeyFrameGraph::vertex_descriptor,kfid> kfVtxInvMap;
	std::mutex covisibilityGraphMtx;

	// End KeyFrame Graph

	/*
	 * Information about this map
	 */
	mapKeyValueInfo keyValueInfo;

	std::vector<double> mScaleFactors;
	std::vector<double> mLevelSigma2;

	/*
	 * Image Database Part
	 */
	ORBVocabulary myVoc;
	std::map<DBoW2::WordId, std::set<kfid> > invertedKeywordDb;
	std::map<kfid, DBoW2::BowVector> BoWList;
	std::map<kfid, DBoW2::FeatureVector> FeatVecList;

};

} /* namespace Vmml */

#endif /* VMML_CORE_VISIONMAP_H_ */
