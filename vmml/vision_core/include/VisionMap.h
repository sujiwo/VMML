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
#include "ORBVocabulary.h"


namespace Vmml {


class VisionMap
{
public:
	VisionMap();
	virtual ~VisionMap();

	// Information about this map
	typedef std::map<std::string,std::string> mapKeyValueInfo;

protected:

	std::mutex *keyframeInvIdx_mtx;
	std::map<kfid, KeyFrame::Ptr> keyframeInvIdx;

	std::mutex *mappointInvIdx_mtx;
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

	void updateCovisibilityGraph(const kfid k);

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
