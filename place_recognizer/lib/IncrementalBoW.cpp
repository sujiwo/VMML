/*
 * IncrementalBoW.cpp
 *
 *  Created on: Sep 23, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <set>
#include <mutex>
#include <boost/dynamic_bitset.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "IncrementalBoW.h"


using namespace std;


namespace PlaceRecognizer {

BinaryDescriptor::BinaryDescriptor(const cv::Mat& desc)
{
	assert(desc.type() == CV_8U);

	size_in_bytes_ = static_cast<unsigned>(desc.cols);
	size_in_bits_ = size_in_bytes_ * 8;
	bits_ = new unsigned char[size_in_bytes_];

	// Creating the descriptor
	const unsigned char* chars = desc.ptr<unsigned char>(0);
	memcpy(bits_, chars, sizeof(unsigned char) * size_in_bytes_);
}

BinaryDescriptor::BinaryDescriptor(const BinaryDescriptor& bd) :
	size_in_bytes_(bd.size_in_bytes_),
	size_in_bits_(bd.size_in_bits_)
{
	bits_ = new unsigned char[size_in_bytes_];
	memcpy(bits_, bd.bits_, sizeof(unsigned char) * size_in_bytes_);
}


BinaryDescriptor::~BinaryDescriptor()
{
	delete [] bits_;
}

cv::Mat BinaryDescriptor::toCvMat() const
{
	cv::Mat m = cv::Mat::zeros(1, size_in_bytes_, CV_8U);
	unsigned char* d = m.ptr<unsigned char>(0);
	memcpy(d, bits_, sizeof(unsigned char) * size_in_bytes_);
	return m.clone();
}

std::string BinaryDescriptor::toString() const
{
	boost::dynamic_bitset<> b(size_in_bits_);
	for (unsigned i = 0; i < size_in_bytes_; i++) {
		unsigned char cur = bits_[i];
		int offset = i * 8;
		for (int bit = 0; bit < 8; bit++) {
			b[offset] = cur & 1;
			offset++;   // Move to next bit in b
			cur >>= 1;  // Move to next bit in array
		}
	}

	std::string st;
	to_string(b, st);
	return st;
}


BinaryTreeNode::BinaryTreeNode() :
	is_leaf_(false),
	is_bad_(false),
	desc_(nullptr),
	root_(nullptr)
{}


BinaryTreeNode::BinaryTreeNode(
	const bool leaf,
	BinaryDescriptor::Ptr desc,
	BinaryTreeNode::Ptr root) :
	is_leaf_(leaf),
	is_bad_(false),
	desc_(desc),
	root_(root)
{}


BinaryTree::BinaryTree(BinaryDescriptor::SetPtr dset,
                       const unsigned tree_id,
                       const unsigned k,
                       const unsigned s) :
  dset_(dset),
  tree_id_(tree_id),
  root_(nullptr),
  k_(k),
  s_(s),
  k_2_(k_ / 2)
{
	srand(time(NULL));
	buildTree();
}


BinaryTree::~BinaryTree()
{ deleteTree(); }


int
BinaryTree::getDepth() const
{
	int depth=0, maxd=0;
	BinaryTreeNode::Ptr parent=root_, child;

	while(true) {
		if (parent->ch_nodes_.size()==0) {

		}
		else {

		}
		if (parent==root_ and child==root_ and depth==0)
			break;
	}

	return depth;
}


void BinaryTree::buildTree()
{
	// Deleting the previous tree, if exists any
	deleteTree();

	degraded_nodes_ = 0;
	nvisited_nodes_ = 0;

	// Creating the root node
	root_ = std::make_shared<BinaryTreeNode>();
	nset_.insert(root_);

	// Generating a new set with the descriptor's ids
	BinaryDescriptor::Set descs = *dset_;

	buildNode(descs, root_);
}


void
BinaryTree::buildNode(BinaryDescriptor::Set dset, BinaryTreeNode::Ptr root)
{
	// Validate if this should be a leaf node
	if (dset.size() < s_) {
		// We set the previous node as a leaf
		root->setLeaf(true);

		// Adding descriptors as leaf nodes
		for (auto it = dset.begin(); it != dset.end(); it++) {
			BinaryDescriptor::Ptr d = *it;
			root->addChildDescriptor(d);

			// Storing the reference of the node where the descriptor hangs
			desc_to_node_[d] = root;
		}
	} else {
		// This node should be split
		// Randomly selecting the new centers
		vector<BinaryDescriptor::Ptr> new_centers;
		vector<BinaryDescriptor::Set> assoc_descs(k_);

		for (uint i = 0; i < k_; i++) {
			// Selecting a new center
			BinaryDescriptor::Ptr desc = *std::next(dset.begin(),
											  rand() % dset.size());
			new_centers.push_back(desc);
			assoc_descs[i].insert(desc);
			dset.erase(desc);
		}

		// Associating the remaining descriptors to the new centers
		for (auto it = dset.begin(); it != dset.end(); it++) {
			BinaryDescriptor::Ptr d = *it;
			int best_center = -1;
			double min_dist = DBL_MAX;
			for (unsigned i = 0; i < k_; i++) {
				double dist = BinaryDescriptor::distHamming(*d, *(new_centers[i]));

				if (dist < min_dist) {
				  min_dist = dist;
				  best_center = i;
				}
			}

			assert(best_center != -1);
			assoc_descs[best_center].insert(d);
		}
		dset.clear();

		// Creating a new tree node for each new cluster
		for (unsigned i = 0; i < k_; i++) {
			BinaryTreeNode::Ptr node = std::make_shared<BinaryTreeNode>(
				false,
				new_centers[i],
				root);

			// Linking this node with its root
			root->addChildNode(node);

			// Storing the reference to this node
			nset_.insert(node);

			// Recursively apply the algorithm
			buildNode(assoc_descs[i], node);
		}
	}
}

void BinaryTree::deleteTree()
{
	if (nset_.size() > 0) {
		nset_.clear();
		desc_to_node_.clear();

		// Invalidating last reference to root
		root_ = nullptr;
	}
}

unsigned BinaryTree::traverseFromRoot(BinaryDescriptor::Ptr q,
                                  NodeQueue::Ptr pq,
                                  DescriptorQueue::Ptr r)
const
{
//	nvisited_nodes_ = 0;
	traverseFromNode(q, root_, pq, r);
//	return nvisited_nodes_;
}

void BinaryTree::traverseFromNode(BinaryDescriptor::Ptr q,
                                  BinaryTreeNode::Ptr n,
                                  NodeQueue::Ptr pq,
                                  DescriptorQueue::Ptr r)
const
{
//	nvisited_nodes_++;
	// If its a leaf node, the search ends
	if (n->isLeaf()) {
		// Adding points to R
		std::unordered_set<BinaryDescriptor::Ptr>* descs =
				n->getChildrenDescriptors();
		for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
			BinaryDescriptor::Ptr d = *it;
			double dist = BinaryDescriptor::distHamming(*q, *d);
			DescriptorQueueItem item(dist, d);
			r->push(item);
		}
	} else {
		// Search continues
		std::unordered_set<BinaryTreeNode::Ptr>* nodes = n->getChildrenNodes();
		int best_node = -1;
		double min_dist = DBL_MAX;

		// Computing distances to nodes
		std::vector<NodeQueueItem> items;
		unsigned node_id = 0;
		// Computing distances to nodes
		for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
			BinaryTreeNode::Ptr bn = *it;
			double dist = bn->distance(q);
			NodeQueueItem item(dist, tree_id_, bn);
			items.push_back(item);

			if (dist < min_dist) {
				min_dist = dist;
				best_node = node_id;
			}

			node_id++;
		}

		assert(best_node != -1);

		// Adding remaining nodes to pq
		for (unsigned i = 0; i < items.size(); i++) {
			// Is it the best node?
			if (i == static_cast<unsigned>(best_node)) {
				continue;
			}

			pq->push(items[i]);
		}

		// Traversing the best node
		traverseFromNode(q, items[best_node].node, pq, r);
	}
}


BinaryTreeNode::Ptr
BinaryTree::searchFromRoot(BinaryDescriptor::Ptr q)
const
{
	return searchFromNode(q, root_);
}


BinaryTreeNode::Ptr
BinaryTree::searchFromNode(BinaryDescriptor::Ptr q, BinaryTreeNode::Ptr n)
const
{
	// If it's a leaf node, the search ends
	if (n->isLeaf()) {
		// This is the node where this descriptor should be included
		return n;
	} else {
		// Search continues
		unordered_set<BinaryTreeNode::Ptr>* nodes = n->getChildrenNodes();
		int best_node = -1;
		double min_dist = DBL_MAX;

		// Computing distances to nodes
		std::vector<BinaryTreeNode::Ptr> items;
		// Computing distances to nodes
		for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
			BinaryTreeNode::Ptr bn = *it;
			items.push_back(bn);
			double dist = bn->distance(q);

			if (dist < min_dist) {
				min_dist = dist;
				best_node = static_cast<int>(items.size()) - 1;
			}
		}

		assert(best_node != -1);

		// Searching in the best node
		return searchFromNode(q, items[best_node]);
	}
}


void BinaryTree::addDescriptor(BinaryDescriptor::Ptr q)
{
	BinaryTreeNode::Ptr n = searchFromRoot(q);
	assert(n->isLeaf());
	if (n->childDescriptorSize() + 1 < s_) {
		// There is enough space at this node for this descriptor, so we add it
		n->addChildDescriptor(q);
		// Storing the reference of the node where the descriptor hangs
		desc_to_node_[q] = n;
	} else {
		// This node should be split
		n->setLeaf(false);

		// Gathering the current descriptors
		std::unordered_set<BinaryDescriptor::Ptr>* descs =
				n->getChildrenDescriptors();
		BinaryDescriptor::Set set;
		for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
			BinaryDescriptor::Ptr d = *it;
			set.insert(d);
		}
		set.insert(q);  // Adding the new descriptor to the set

		// Rebuilding this node
		buildNode(set, n);
	}
}


void BinaryTree::deleteDescriptor(BinaryDescriptor::Ptr q)
{
	// We get the node where the descriptor is stored
	BinaryTreeNode::Ptr node = desc_to_node_[q];
	assert(node->isLeaf());

	// We remove q from the node
	node->deleteChildDescriptor(q);

	if (node->childDescriptorSize() > 0) {
		// We select a new center, if required
		if (node->getDescriptor() == q) {
			// Selecting a new center
			node->selectNewCenter();
		}
	} else {
		// Otherwise, we need to remove the node
		BinaryTreeNode::Ptr parent = node->getRoot();
		parent->deleteChildNode(node);
		nset_.erase(node);

		deleteNodeRecursive(parent);
	}

	desc_to_node_.erase(q);
}


void BinaryTree::deleteNodeRecursive(BinaryTreeNode::Ptr n)
{
	assert(!n->isLeaf());
	// Validating if this node is degraded
	if (n->childNodeSize() < k_2_ && !n->isBad()) {
		degraded_nodes_++;
		n->setBad(true);
	}

	if (n->childNodeSize() == 0 && n != root_) {
		// We remove this node
		BinaryTreeNode::Ptr parent = n->getRoot();
		parent->deleteChildNode(n);
		nset_.erase(n);

		deleteNodeRecursive(parent);
	}
}


void BinaryTree::printTree()
{
	printNode(root_);
}


void BinaryTree::printNode(BinaryTreeNode::Ptr n)
{
	std::cout << "---" << std::endl;
	std::cout << "Node: " << n << std::endl;
	std::cout << (n->isLeaf() ? "Leaf" : "Node") << std::endl;
	std::cout << "Descriptor: " << n->getDescriptor() << std::endl;
	if (n->isLeaf()) {
		std::cout << "Children descriptors: " <<
			n->childDescriptorSize() << std::endl;
	} else {
		std::cout << "Children nodes: " << n->childNodeSize() << std::endl;
		std::unordered_set<BinaryTreeNode::Ptr>* nodes = n->getChildrenNodes();
		for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
			printNode(*it);
		}
	}
}


IncrementalBoW::IncrementalBoW(
	const unsigned k,
	const unsigned s,
	const unsigned t,
	const MergePolicy merge_policy,
	const bool purge_descriptors,
	const unsigned min_feat_apps) :
	k_(k),
	s_(s),
	t_(t),
	init_(false),
	nimages_(0),
	ndesc_(0),
	merge_policy_(merge_policy),
	purge_descriptors_(purge_descriptors),
	min_feat_apps_(min_feat_apps)
{
	// Validating the corresponding parameters
	assert(k_ > 1);
	assert(k_ < s_);
	assert(min_feat_apps > 0);
}


void IncrementalBoW::addImage(
	const unsigned image_id,
	const std::vector<cv::KeyPoint>& kps,
	const cv::Mat& descs)
{
	// Creating the set of BinaryDescriptors
	for (int i = 0; i < descs.rows; i++) {
		// Creating the corresponding descriptor
		cv::Mat desc = descs.row(i);
		BinaryDescriptor::Ptr d = std::make_shared<BinaryDescriptor>(desc);
		insertDescriptor(d);

		// Creating the inverted index item
		InvIndexItem item;
		item.image_id = image_id;
		item.pt = kps[i].pt;
		item.dist = 0.0;
		item.kp_ind = i;
		inv_index_[d].push_back(item);
	}

	// If the trees are not initialized, we build them
	if (!init_) {
		assert(static_cast<int>(k_) < descs.rows);
		initTrees();
		init_ = true;
	}

	// Deleting unstable features
	if (purge_descriptors_) {
		purgeDescriptors(image_id);
	}

	nimages_++;
}


void IncrementalBoW::addImage2(const unsigned image_id,
		   const std::vector<cv::KeyPoint>& kps,
		   const cv::Mat& descs)
{
	vector<vector<cv::DMatch>> matches_feats;
	vector<cv::DMatch> realMatches;

	// XXX: Parallelize this
	searchDescriptors(descs, matches_feats, 2, 64);

	for (uint m=0; m<matches_feats.size(); ++m) {
		if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8)
			realMatches.push_back(matches_feats[m][0]);
	}
	addImage(image_id, kps, descs, realMatches);
}


void IncrementalBoW::addImage(
	const imageId image_id,
	const std::vector<cv::KeyPoint>& kps,
	const cv::Mat& descs,
	const std::vector<cv::DMatch>& matches)
{
	// --- Adding new features
	// All features
	set<int> points;
	for (unsigned feat_ind = 0; feat_ind < kps.size(); feat_ind++) {
		points.insert(feat_ind);
	}

	// Matched features
	std::set<int> matched_points;
	for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
		matched_points.insert(matches[match_ind].queryIdx);
	}

	// Computing the difference
	std::set<int> diff;
	std::set_difference(points.begin(), points.end(),
					  matched_points.begin(), matched_points.end(),
					  std::inserter(diff, diff.end()));

	// Inserting new features into the index.
	for (auto it = diff.begin(); it != diff.end(); it++) {
		int index = *it;
		cv::Mat desc = descs.row(index);
		BinaryDescriptor::Ptr d = std::make_shared<BinaryDescriptor>(desc);
		insertDescriptor(d);

		// Creating the inverted index item
		InvIndexItem item;
		item.image_id = image_id;
		item.pt = kps[index].pt;
		item.dist = 0.0;
		item.kp_ind = index;
		inv_index_[d].push_back(item);
	}

	// --- Updating the matched descriptors into the index
	for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
		int qindex = matches[match_ind].queryIdx;
		int tindex = matches[match_ind].trainIdx;

		BinaryDescriptor::Ptr q_d = std::make_shared<BinaryDescriptor> (descs.row(qindex));
		BinaryDescriptor::Ptr t_d = id_to_desc_[tindex];

		// Merge and replace according to the merging policy
		if (merge_policy_ == MERGE_POLICY_AND) {
			*t_d &= *q_d;
		} else if (merge_policy_ == MERGE_POLICY_OR) {
			*t_d |= *q_d;
		}

		// Creating the inverted index item
		InvIndexItem item;
		item.image_id = image_id;
		item.pt = kps[qindex].pt;
		item.dist = matches[match_ind].distance;
		item.kp_ind = qindex;
		inv_index_[t_d].push_back(item);
	}

	// Deleting unstable features
	if (purge_descriptors_) {
		purgeDescriptors(image_id);
	}

	nimages_++;
}


void IncrementalBoW::searchImages(
	const cv::Mat& descs,
	const std::vector<cv::DMatch>& gmatches,
	vector<ImageMatch> &img_matches)
const
{
	std::map<imageId, double> imgMatchesMap;

	// Counting the number of each word in the current document
	std::unordered_map<int, int> nwi_map;
	for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
		int train_idx = gmatches[match_index].trainIdx;
		// Updating nwi_map, number of occurrences of a word in an image.
		if (nwi_map.count(train_idx)) {
			nwi_map[train_idx]++;
		} else {
			nwi_map[train_idx] = 1;
		}
	}

	// We process all the matchings again to increase the scores
	for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
		int train_idx = gmatches[match_index].trainIdx;
		auto desc = id_to_desc_.at(train_idx);

		// Computing the TF term
		double tf = static_cast<double>(nwi_map[train_idx]) / descs.rows;

		// Computing the IDF term
		std::unordered_set<unsigned> nw;
		for (uint i=0; i<inv_index_.at(desc).size(); ++i) {
			nw.insert(inv_index_.at(desc).at(i).image_id);
		}
		double idf = log(static_cast<double>(nimages_) / nw.size());

		// Computing the final TF-IDF weighting term
		double tfidf = tf * idf;

		for (uint i=0; i<inv_index_.at(desc).size(); ++i) {
			int im = inv_index_.at(desc).at(i).image_id;
			try {
				imgMatchesMap.at(im) += tfidf;
			} catch (out_of_range &e) {
				imgMatchesMap[im] = tfidf;
			}
		}
	}

	img_matches.clear();
	for (auto &mt: imgMatchesMap) {
		ImageMatch candidate(mt.first, mt.second);
		img_matches.push_back(candidate);
	}
	std::sort(img_matches.begin(), img_matches.end());
}


void IncrementalBoW::initTrees()
{
	// Creating the trees
	BinaryDescriptor::SetPtr dset_ptr = std::make_shared<BinaryDescriptor::Set>(dset_);

	for (unsigned i = 0; i < t_; i++) {
		BinaryTree::Ptr tree_ptr = make_shared<BinaryTree>(dset_ptr, i, k_, s_);
		trees_.push_back(tree_ptr);
	}
}


void IncrementalBoW::searchDescriptors(
	const cv::Mat& descs,
	std::vector<std::vector<cv::DMatch>> &matches,
	const unsigned knn,
	const unsigned checks)
const
{
	mutex matchResultMtx;
	matches.clear();

#pragma omp parallel for
	for (int i = 0; i < descs.rows; i++) {
		// Creating the corresponding descriptor
		cv::Mat desc = descs.row(i);
		BinaryDescriptor::Ptr d = std::make_shared<BinaryDescriptor>(desc);

		// Searching the descriptor in the index
		vector<BinaryDescriptor::Ptr> neighs;
		vector<double> dists;
		searchDescriptor(d, &neighs, &dists, knn, checks);

		// Translating the resulting matches to CV structures
		std::vector<cv::DMatch> des_match;
		for (unsigned j = 0; j < neighs.size(); j++) {
			cv::DMatch match;
			match.queryIdx = i;
			match.trainIdx = static_cast<int>(desc_to_id_.at(neighs[j]));
			match.imgIdx = static_cast<int>(inv_index_.at(neighs[j]).at(0).image_id);
			match.distance = dists[j];
			des_match.push_back(match);
		}

		matchResultMtx.lock();
		matches.push_back(des_match);
		matchResultMtx.unlock();
	}
}


void IncrementalBoW::deleteDescriptor(const unsigned desc_id)
{
	BinaryDescriptor::Ptr d = id_to_desc_[desc_id];
	// Clearing the descriptor
	deleteDescriptor(d);
}


void IncrementalBoW::searchDescriptor(
	BinaryDescriptor::Ptr q,
	std::vector<BinaryDescriptor::Ptr>* neigh,
	std::vector<double>* distances,
	unsigned knn,
	unsigned checks)
const
{
	uint points_searched = 0;
	NodePriorityQueue pq;
	DescriptorQueue r;

	// Initializing search structures
	std::vector<NodeQueue::Ptr> pqs;
	std::vector<DescriptorQueue::Ptr> rs;
	for (uint i = 0; i < trees_.size(); i++) {
		NodeQueue::Ptr tpq = std::make_shared<NodeQueue>();
		pqs.push_back(tpq);

		DescriptorQueue::Ptr tr = std::make_shared<DescriptorQueue>();
		rs.push_back(tr);
	}

	// Searching in the trees
//	#pragma omp parallel for
		for (uint i = 0; i < trees_.size(); i++) {
			trees_[i]->traverseFromRoot(q, pqs[i], rs[i]);
		}

	//  Gathering results from each individual search
	std::unordered_set<BinaryDescriptor::Ptr> already_added;
	for (uint i = 0; i < trees_.size(); i++) {
		// Obtaining descriptor nodes
		uint r_size = rs[i]->size();
		for (uint j = 0; j < r_size; j++) {
			DescriptorQueueItem r_item = rs[i]->get(j);
			std::pair<std::unordered_set<BinaryDescriptor::Ptr>::iterator,
					bool > result;
			result = already_added.insert(r_item.desc);
			if (result.second) {
				r.push(r_item);
				points_searched++;
			}
		}
	}

	// Continuing the search if not enough descriptors have been checked
	if (points_searched < checks) {
		// Gathering the next nodes to search
		for (uint i = 0; i < trees_.size(); i++) {
			// Obtaining priority queue nodes
			uint pq_size = pqs[i]->size();
			for (uint j = 0; j < pq_size; j++) {
				pq.push(pqs[i]->get(j));
			}
		}

		NodePriorityQueuePtr pq_ptr = std::make_shared<NodePriorityQueue>(pq);
		while (points_searched < checks && !pq.empty()) {
			// Get the closest node to continue the search
			NodeQueueItem n = pq.top();
			pq.pop();

			// Searching in the node
			NodeQueue::Ptr tpq = std::make_shared<NodeQueue>();
			DescriptorQueue::Ptr tr = std::make_shared<DescriptorQueue>();
			trees_[n.tree_id]->traverseFromNode(q, n.node, tpq, tr);

			// Adding new nodes to search to PQ
			for (uint i = 0; i < tpq->size(); i++) {
			pq.push(tpq->get(i));
			}

			for (uint j = 0; j < tr->size(); j++) {
				DescriptorQueueItem r_item = tr->get(j);
				std::pair<std::unordered_set<BinaryDescriptor::Ptr>::iterator, bool> result;
				result = already_added.insert(r_item.desc);
				if (result.second) {
					r.push(r_item);
					points_searched++;
				}
			}
		}
	}
	r.sort();

	// Returning the required number of descriptors descriptors
	neigh->clear();
	distances->clear();
	uint ndescs = std::min(knn, r.size());
	for (unsigned i = 0; i < ndescs; i++) {
		DescriptorQueueItem d = r.get(i);
		neigh->push_back(d.desc);
		distances->push_back(d.dist);
	}
}


void IncrementalBoW::insertDescriptor(BinaryDescriptor::Ptr q)
{
	dset_.insert(q);
	desc_to_id_[q] = ndesc_;
	id_to_desc_[ndesc_] = q;
	ndesc_++;
	recently_added_.push_back(q);

	// Indexing the descriptor inside each tree
	if (init_) {
	#pragma omp parallel for
		for (unsigned i = 0; i < trees_.size(); i++) {
			trees_[i]->addDescriptor(q);
		}
	}
}


void IncrementalBoW::deleteDescriptor(BinaryDescriptor::Ptr q)
{
	// Deleting the descriptor from each tree
	if (init_) {
	#pragma omp parallel for
		for (unsigned i = 0; i < trees_.size(); i++) {
			trees_[i]->deleteDescriptor(q);
		}
	}

	dset_.erase(q);
	unsigned desc_id = desc_to_id_[q];
	desc_to_id_.erase(q);
	id_to_desc_.erase(desc_id);
	inv_index_.erase(q);
}


void IncrementalBoW::getMatchings(
      const std::vector<cv::KeyPoint>& query_kps,
      const std::vector<cv::DMatch>& matches,
      std::unordered_map<unsigned, ImageMatch::PointMatches>* point_matches)
{
	for (unsigned i = 0; i < matches.size(); i++) {
		// Getting the query point
		int qid = matches[i].queryIdx;
		cv::Point2f qpoint = query_kps[qid].pt;

		// Processing the train points
		int tid = matches[i].trainIdx;
		BinaryDescriptor::Ptr desc_ptr = id_to_desc_[static_cast<uint64_t>(tid)];
		for (unsigned j = 0; j < inv_index_[desc_ptr].size(); j++) {
			InvIndexItem item = inv_index_[desc_ptr][j];
			unsigned im_id = item.image_id;
			cv::Point2f tpoint = item.pt;

			(*point_matches)[im_id].query.push_back(qpoint);
			(*point_matches)[im_id].train.push_back(tpoint);
		}
	}
}


void IncrementalBoW::purgeDescriptors(const uint curr_img)
{
	auto it = recently_added_.begin();

	while (it != recently_added_.end()) {
		BinaryDescriptor::Ptr desc = *it;
		// We assess if at least three images have passed since creation
		if ((curr_img - inv_index_[desc][0].image_id) > 1) {
			// If so, we assess if the feature has been seen at least twice
			if (inv_index_[desc].size() < min_feat_apps_) {
				deleteDescriptor(desc);
			}

			it = recently_added_.erase(it);
		} else {
			// This descriptor should be maintained in the list
			it++;
		}
	}
}


void
IncrementalBoW::saveToDisk(const std::string &f) const
{
	fstream indexFileFd;
	indexFileFd.open(f, fstream::out | fstream::trunc);
	if (!indexFileFd.is_open())
		throw runtime_error("Unable to create map file");
	boost::archive::binary_oarchive indexStore(indexFileFd);

	indexStore << *this;

	indexFileFd.close();
}


void
IncrementalBoW::loadFromDisk(const std::string &f)
{
	fstream indexFileFd;
	indexFileFd.open(f, fstream::in);
	if (!indexFileFd.is_open())
		throw runtime_error("Unable to create map file");
	boost::archive::binary_iarchive indexStore(indexFileFd);

	indexStore >> *this;

	indexFileFd.close();
}


} /* namespace PlaceRecognizer */
