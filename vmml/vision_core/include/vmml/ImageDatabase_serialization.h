/*
 * ImageDatabase_serialization.h
 *
 *  Created on: Dec 24, 2019
 *      Author: sujiwo
 */

#ifndef _IMAGEDATABASE_SERIALIZATION_H_
#define _IMAGEDATABASE_SERIALIZATION_H_

#include <boost/serialization/unordered_set.hpp>
#include "ImageDatabase.h"

namespace Vmml {


template <class Archive>
void BinaryDescriptor::serialize(Archive &ar, const unsigned int v)
{
	ar & size_in_bytes_ & size_in_bits_;
	ar & boost::serialization::make_array(bits_, size_in_bytes_);
}


template<class Archive>
void BinaryTreeNode::serialize(Archive &ar, const uint v)
{
	ar
		& is_leaf_
		& is_bad_
		& desc_
		& root_
		& ch_nodes_
		& ch_descs_;
}


template<class Archive>
void BinaryTree::serialize(Archive &ar, const unsigned int v)
{
	ar  & tree_id_
		& root_
		& k_
		& s_
		& k_2_
		& nset_
		& desc_to_node_
		& degraded_nodes_
		& nvisited_nodes_;
}



template<class Archive>
void InvIndexItem::serialize(Archive & ar, const unsigned int version)
{
	ar & image_id
	& pt
	& dist
	& kp_ind;
}



/*
 * Implementation of serialization for ImageDatabase
 */
template<class Archive>
void ImageDatabase::save(Archive &ar, const unsigned int v) const
{
	ar << k_;
	ar << s_;
	ar << t_;
	ar << init_;
	ar << nimages_;
	ar << ndesc_;
	ar << merge_policy_;
	ar << purge_descriptors_;
	ar << min_feat_apps_;

	// Descriptors
	std::vector<cv::Mat> descriptorSerialized;
	std::map<BinaryDescriptor::Ptr, uint64> descriptorPtrId;
	std::unordered_map<uint64, std::vector<InvIndexItem>> invIndexEncoded;
	std::unordered_map<uint64, uint64> descriptorIdEncodedToRealDescId;

	encodeDescriptors(descriptorSerialized, descriptorPtrId, invIndexEncoded, descriptorIdEncodedToRealDescId);
	ar << descriptorSerialized;
	ar << invIndexEncoded;
	ar << descriptorIdEncodedToRealDescId;

	// Trees
	ar << trees_.size();
	for (int i=0; i<trees_.size(); i++) {
		ar << *trees_[i];

		set<uint64> dsetEnc;
		vector<BinaryTreeNode::BinaryTreeNodeData_> nodeData;
		map<uint64, uint64> desc_to_node_enc;
		uint64 rootId;
		std::unordered_map<uint64, set<uint64>> nodesChilds;
		std::unordered_map<uint64, set<uint64>> nodesChildDescriptors;
		trees_[i]->encode(descriptorPtrId, dsetEnc, nodeData, desc_to_node_enc, rootId, nodesChilds, nodesChildDescriptors);
		ar << dsetEnc
			<< nodeData
			<< desc_to_node_enc
			<< rootId
			<< nodesChilds
			<< nodesChildDescriptors;
	}
}


template<class Archive>
void ImageDatabase::load(Archive &ar, const unsigned int v)
{
	ar >> k_;
	ar >> s_;
	ar >> t_;
	ar >> init_;
	ar >> nimages_;
	ar >> ndesc_;
	ar >> merge_policy_;
	ar >> purge_descriptors_;
	ar >> min_feat_apps_;

	// Descriptors
	vector<cv::Mat> descriptorSerialized;
	std::map<BinaryDescriptor::Ptr, uint64> descriptorPtrId;
	std::unordered_map<uint64, std::vector<InvIndexItem>> invIndexEncoded;
	std::unordered_map<uint64, uint64> descriptorIdEncodedToRealDescId;
	ar >> descriptorSerialized;
	ar >> invIndexEncoded;
	ar >> descriptorIdEncodedToRealDescId;
	decodeDescriptors(descriptorSerialized, descriptorPtrId, invIndexEncoded, descriptorIdEncodedToRealDescId);

	uint numOfTrees;
	ar >> numOfTrees;
	for (int i=0; i<numOfTrees; i++) {
		auto iTree = std::make_shared<BinaryTree>();
		ar >> *iTree;
		trees_.push_back(iTree);
	}
}


}		// namespace Vmml

#endif /* _IMAGEDATABASE_SERIALIZATION_H_ */
