/*
 * ImageDatabase_serialization.h
 *
 *  Created on: Dec 24, 2019
 *      Author: sujiwo
 */

#ifndef _INCREMENTALBOW_SERIALIZATION_H_
#define _INCREMENTALBOW_SERIALIZATION_H_

#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include "cvobj_serialization.h"
#include "IncrementalBoW.h"


namespace PlaceRecognizer {


template <class Archive>
void BinaryDescriptor::serialize(Archive &ar, const unsigned int v)
{
	ar & size_in_bytes_ & size_in_bits_;
    if (Archive::is_loading::value)
    {
        assert(bits_ == nullptr);
        bits_ = new unsigned char[size_in_bytes_];
    }
	ar & boost::serialization::make_array<unsigned char>(bits_, size_in_bytes_);
}


template<class Archive>
void BinaryTreeNode::serialize(Archive &ar, const uint v)
{
	ar	& is_leaf_;
	ar	& is_bad_;
	ar	& desc_;
	ar	& root_;
	ar	& ch_nodes_;
	ar	& ch_descs_;
}


template<class Archive>
void BinaryTree::serialize(Archive &ar, const unsigned int v)
{
	ar  & tree_id_;
	ar	& root_;
	ar	& k_;
	ar	& s_;
	ar	& k_2_;
	ar	& nset_;
	ar	& desc_to_node_;
	ar	& degraded_nodes_;
	ar	& nvisited_nodes_;
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
void IncrementalBoW::save(Archive &ar, const unsigned int v) const
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

	ar << dset_;
	ar << inv_index_;
	ar << desc_to_id_;
	ar << recently_added_;

	ar << keyframeIdToBag;

	for (auto &t: trees_) {
		ar << *t;
	}
}


template<class Archive>
void IncrementalBoW::load(Archive &ar, const unsigned int v)
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

	ar >> dset_;
	ar >> inv_index_;
	ar >> desc_to_id_;
	ar >> recently_added_;

	id_to_desc_.clear();
	for (auto &mp: desc_to_id_) {
		id_to_desc_[mp.second] = mp.first;
	}

	// Run integrity check
	for (auto &descPtr: dset_) {
		try {
			auto im = inv_index_.at(descPtr);
		} catch (std::out_of_range &e) {
			std::cerr << "Unpaired descriptor found" << std::endl;
		}
	}

	ar >> keyframeIdToBag;

	trees_.clear();
	for (int i=0; i<t_; i++) {
		BinaryTree::Ptr vtree(new BinaryTree);
		ar >> *vtree;
		trees_.push_back(vtree);
	}
}


}		// namespace PlaceRecognizer

#endif /* _INCREMENTALBOW_SERIALIZATION_H_ */
