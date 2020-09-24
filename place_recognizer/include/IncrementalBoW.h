/*
 * IncrementalBoW.h
 *
 *  Created on: Sep 23, 2020
 *      Author: sujiwo
 */

#ifndef PLACE_RECOGNIZER_INCREMENTALBOW_H_
#define PLACE_RECOGNIZER_INCREMENTALBOW_H_


#include <memory>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <string>
#include <list>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <opencv2/features2d.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>


namespace PlaceRecognizer {

typedef uint32_t imageId;


class BinaryDescriptor
{
public:
	typedef std::shared_ptr<BinaryDescriptor> Ptr;
	typedef std::unordered_set<BinaryDescriptor::Ptr> Set;
	typedef std::shared_ptr<BinaryDescriptor::Set> SetPtr;

	explicit BinaryDescriptor():
		bits_(nullptr), size_in_bits_(0), size_in_bytes_(0)
	{}

	explicit BinaryDescriptor(const cv::Mat& desc);
	explicit BinaryDescriptor(const BinaryDescriptor& bd);

	// Destructor
	virtual ~BinaryDescriptor();

	// Methods
	inline void set(int nbit) {
		// Detecting the correct byte
		int nbyte = nbit / 8;
		int nb = 7 - (nbit % 8);

		// Setting the bit
		bits_[nbyte] |= 1 << nb;
	}

	inline void reset(int nbit)
	{
		// Detecting the correct byte
		int nbyte = nbit / 8;
		int nb = 7 - (nbit % 8);

		// Setting the bit
		bits_[nbyte] &= ~(1 << nb);
	}

	inline int size() const
	{ return static_cast<int>(size_in_bits_); }

	inline static double distHamming(const BinaryDescriptor& a, const BinaryDescriptor& b)
	{
		int hamming = cv::hal::normHamming(a.bits_, b.bits_, a.size_in_bytes_);
		return static_cast<double>(hamming);
	}

	// Operator overloading
	inline bool operator==(const BinaryDescriptor& d) const
	{
		int hamming = cv::hal::normHamming(bits_, d.bits_, size_in_bytes_);
		return hamming == 0;
	}

	inline bool operator!=(const BinaryDescriptor& d) const
	{
		int hamming = cv::hal::normHamming(bits_, d.bits_, size_in_bytes_);
		return hamming != 0;
	}

	inline BinaryDescriptor& operator=(const BinaryDescriptor& other)
	{
		// Clearing previous memory
		if (bits_ != nullptr) {
		  delete [] bits_;
		}

		// Allocating new memory
		size_in_bits_ = other.size_in_bits_;
		size_in_bytes_ = other.size_in_bytes_;
		bits_ = new unsigned char[size_in_bytes_];
		memcpy(bits_, other.bits_, sizeof(unsigned char) * size_in_bytes_);

		return *this;
	}

	inline BinaryDescriptor& operator&=(const BinaryDescriptor& other)
	{
		unsigned size = other.size_in_bytes_;
		for (unsigned i = 0; i < size; i++) {
		  bits_[i] = bits_[i] & other.bits_[i];
		}

		return *this;
	}

	inline BinaryDescriptor& operator|=(const BinaryDescriptor& other)
	{
		unsigned size = other.size_in_bytes_;
		for (unsigned i = 0; i < size; i++) {
			bits_[i] = bits_[i] | other.bits_[i];
		}

		return *this;
	}

	cv::Mat toCvMat() const;
	std::string toString() const;

	unsigned char* bits_;
	unsigned int size_in_bytes_;
	unsigned int size_in_bits_;

	// Serialization support
	template <class Archive>
	void serialize(Archive &ar, const unsigned int v);
};


class BinaryTreeNode {
public:

	typedef std::shared_ptr<BinaryTreeNode> Ptr;

	friend class ImageDatabase;
	friend class BinaryTree;

	// Constructors
	BinaryTreeNode();
	explicit BinaryTreeNode(const bool leaf, BinaryDescriptor::Ptr desc = nullptr, BinaryTreeNode::Ptr root = nullptr);

	// Methods
	inline bool isLeaf() {
		return is_leaf_;
	}

	inline void setLeaf(const bool leaf)
	{ is_leaf_ = leaf; }

	inline bool isBad() const
	{ return is_bad_; }

	inline void setBad(const bool bad)
	{ is_bad_ = bad; }

	inline BinaryDescriptor::Ptr getDescriptor()
	{ return desc_; }

	inline void setDescriptor(BinaryDescriptor::Ptr desc)
	{ desc_ = desc; }

	inline BinaryTreeNode::Ptr getRoot()
	{ return root_; }

	inline void setRoot(BinaryTreeNode::Ptr root)
	{ root_ = root;	}

	inline double distance(BinaryDescriptor::Ptr desc)
	{ return BinaryDescriptor::distHamming(*desc_, *desc); }

	inline void addChildNode(BinaryTreeNode::Ptr child)
	{ ch_nodes_.insert(child); }

	inline void deleteChildNode(BinaryTreeNode::Ptr child)
	{ ch_nodes_.erase(child); }

	inline std::unordered_set<BinaryTreeNode::Ptr>* getChildrenNodes()
	{ return &ch_nodes_; }

	inline unsigned childNodeSize() const
	{ return ch_nodes_.size(); }

	inline void addChildDescriptor(BinaryDescriptor::Ptr child)
	{ ch_descs_.insert(child); }

	inline void deleteChildDescriptor(BinaryDescriptor::Ptr child)
	{ ch_descs_.erase(child); }

	inline std::unordered_set<BinaryDescriptor::Ptr>* getChildrenDescriptors()
	{ return &ch_descs_; }

	inline unsigned childDescriptorSize() const
	{ return ch_descs_.size(); }

	inline void selectNewCenter()
	{ desc_ = *std::next(ch_descs_.begin(), rand() % ch_descs_.size()); }

	typedef std::unordered_set<BinaryTreeNode::Ptr> Set;

	template<class Archive>
	void serialize(Archive &ar, const uint v);

private:
	bool is_leaf_;
	bool is_bad_;
	BinaryDescriptor::Ptr desc_;
	BinaryTreeNode::Ptr root_;
	std::unordered_set<BinaryTreeNode::Ptr> ch_nodes_;
	std::unordered_set<BinaryDescriptor::Ptr> ch_descs_;

	struct BinaryTreeNodeData_ {
		bool is_leaf_;
		bool is_bad_;
		uint64 descriptorId;
		uint64 root_;

		template<class Archive>
		void serialize(Archive &ar, const unsigned int version)
		{ ar & is_leaf_ & is_bad_ & descriptorId & root_; }
	};
};


struct NodeQueueItem
{
public:
	inline explicit NodeQueueItem(
		const double d,
		const unsigned id,
		BinaryTreeNode::Ptr n) :
		dist(d),
		tree_id(id),
		node(n)
	{}

	double dist;
	unsigned tree_id;
	BinaryTreeNode::Ptr node;

	inline bool operator<(const NodeQueueItem& item) const
	{ return dist < item.dist; }
};


class NodeQueue {
 public:
  inline void push(const NodeQueueItem& item) {
    items.push_back(item);
  }

  inline NodeQueueItem get(unsigned index) {
    return items[index];
  }

  inline void sort() {
    std::sort(items.begin(), items.end());
  }

  inline unsigned size() {
    return items.size();
  }

  typedef std::shared_ptr<NodeQueue> Ptr;

 private:
  std::vector<NodeQueueItem> items;
};


class CompareNodeQueueItem
{
public:
	inline bool operator()(const NodeQueueItem& a, const NodeQueueItem& b)
	{ return a.dist > b.dist; }
};


typedef std::priority_queue<
	NodeQueueItem,
    std::vector<NodeQueueItem>,
    CompareNodeQueueItem> NodePriorityQueue;
typedef std::shared_ptr<NodePriorityQueue> NodePriorityQueuePtr;


struct DescriptorQueueItem
{
public:
	inline explicit DescriptorQueueItem(const double d, BinaryDescriptor::Ptr bd) :
		dist(d),
		desc(bd)
	{}

	double dist;
	BinaryDescriptor::Ptr desc;

	inline bool operator<(const DescriptorQueueItem& item) const
	{ return dist < item.dist; }
};


class DescriptorQueue
{
	public:
	inline void push(const DescriptorQueueItem& item)
	{ items.push_back(item); }

	inline DescriptorQueueItem get(unsigned index)
	{ return items[index]; }

	inline void sort()
	{ std::sort(items.begin(), items.end()); }

	inline unsigned size() {
	return items.size();
	}

	typedef std::shared_ptr<DescriptorQueue> Ptr;

	private:
	std::vector<DescriptorQueueItem> items;
};


class BinaryTree
{
public:
	typedef std::shared_ptr<BinaryTree> Ptr;

	friend class IncrementalBoW;

	BinaryTree() {}

	// Constructors
	explicit BinaryTree(BinaryDescriptor::SetPtr dset,
					  const unsigned tree_id = 0,
					  const unsigned k = 16,
					  const unsigned s = 150);
	virtual ~BinaryTree();

	// Methods
	void buildTree();
	void deleteTree();
	unsigned traverseFromRoot(BinaryDescriptor::Ptr q,
							NodeQueue::Ptr pq,
							DescriptorQueue::Ptr r) const;
	void traverseFromNode(BinaryDescriptor::Ptr q,
						BinaryTreeNode::Ptr n,
						NodeQueue::Ptr pq,
						DescriptorQueue::Ptr r) const;
	BinaryTreeNode::Ptr searchFromRoot(BinaryDescriptor::Ptr q) const;
	BinaryTreeNode::Ptr searchFromNode(BinaryDescriptor::Ptr q, BinaryTreeNode::Ptr n) const;
	void addDescriptor(BinaryDescriptor::Ptr q);
	void deleteDescriptor(BinaryDescriptor::Ptr q);
	void printTree();

	inline unsigned numDegradedNodes() const
	{ return degraded_nodes_; }

	inline unsigned numNodes() const
	{ return nset_.size(); }

	int getDepth() const;

	/*
	 * Serialization support
	 */
	template<class Archive>
	void serialize(Archive &ar, const unsigned int v);

private:
	BinaryDescriptor::SetPtr dset_;
	unsigned tree_id_;
	BinaryTreeNode::Ptr root_;
	unsigned k_;
	unsigned s_;
	unsigned k_2_;
	BinaryTreeNode::Set nset_;
	std::unordered_map<BinaryDescriptor::Ptr, BinaryTreeNode::Ptr> desc_to_node_;

	// Tree statistics
	unsigned degraded_nodes_;
	unsigned nvisited_nodes_;

	void buildNode(BinaryDescriptor::Set d, BinaryTreeNode::Ptr root);
	void printNode(BinaryTreeNode::Ptr n);
	void deleteNodeRecursive(BinaryTreeNode::Ptr n);

};	// BinaryTree


struct InvIndexItem
{
	InvIndexItem() :
	  image_id(0),
	  pt(0.0f, 0.0f),
	  dist(DBL_MAX),
	  kp_ind(-1) {}

	InvIndexItem(const int id, const cv::Point2f &kp, const double d, const int kp_i = -1) :
		image_id(id),
		pt(kp),
		dist(d),
		kp_ind(kp_i)
	{}

	imageId image_id;
	cv::Point2f pt;
	double dist;
	int kp_ind;

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


struct ImageMatch
{
	ImageMatch() :
		image_id(-1),
		score(0.0) {}

	explicit ImageMatch(const int id, const double sc = 0.0) :
		image_id(id),
		score(sc) {}

	int image_id;
	double score;

	bool operator<(const ImageMatch &lcr) const
	{ return score > lcr.score; }

	struct PointMatches
	{
		std::vector<cv::Point2f> query;
		std::vector<cv::Point2f> train;
	};
};


class IncrementalBoW
{
public:
	friend class boost::serialization::access;

	enum MergePolicy {
	  MERGE_POLICY_NONE,
	  MERGE_POLICY_AND,
	  MERGE_POLICY_OR
	};

	explicit IncrementalBoW(
		const uint k = 16,
		const uint s = 150,
		const uint t = 4,
		const MergePolicy merge_policy = MergePolicy::MERGE_POLICY_NONE,
		const bool purge_descriptors = true,
		const uint min_feat_apps = 3);

	// For initial image
	void addImage(const unsigned image_id,
			   const std::vector<cv::KeyPoint>& kps,
			   const cv::Mat& descs);

	void addImage2(const unsigned image_id,
			   const std::vector<cv::KeyPoint>& kps,
			   const cv::Mat& descs);

	void addImage(const imageId image_id,
			   const std::vector<cv::KeyPoint>& kps,
			   const cv::Mat& descs,
			   const std::vector<cv::DMatch>& matches);

	void searchImages(const cv::Mat& descs,
				   const std::vector<cv::DMatch>& gmatches,
				   std::vector<ImageMatch> &img_matches)
		const;

	void searchDescriptors(
		const cv::Mat& descs,
		std::vector<std::vector<cv::DMatch>> &matches,
		const unsigned knn = 2,
		const unsigned checks = 32)
	const;

	void deleteDescriptor(const unsigned desc_id);

	void getMatchings(const std::vector<cv::KeyPoint>& query_kps,
				   const std::vector<cv::DMatch>& matches,
				   std::unordered_map<unsigned, ImageMatch::PointMatches>* point_matches);

	inline unsigned numImages() const
	{ return nimages_; }

	inline unsigned numDescriptors() const
	{ return dset_.size(); }

	inline void rebuild()
	{
		if (init_) {
			trees_.clear();
			initTrees();
		}
	}

	void saveToDisk(const std::string &f) const;
	void loadFromDisk(const std::string &f);

	std::map<imageId,uint64> keyframeIdToBag;

private:
	BinaryDescriptor::Set dset_;
	unsigned k_;
	unsigned s_;
	unsigned t_;
	unsigned init_;
	unsigned nimages_;
	unsigned ndesc_;
	MergePolicy merge_policy_;
	bool purge_descriptors_;

	// Minimum times a feature has shown up in multiple times before being discarded
	unsigned min_feat_apps_;

	std::unordered_map<BinaryDescriptor::Ptr, std::vector<InvIndexItem> > inv_index_;
	std::unordered_map<BinaryDescriptor::Ptr, uint64_t> desc_to_id_;
	std::unordered_map<uint64_t, BinaryDescriptor::Ptr> id_to_desc_;
	std::list<BinaryDescriptor::Ptr> recently_added_;
	std::vector<BinaryTree::Ptr> trees_;

	void initTrees();

	void searchDescriptor(BinaryDescriptor::Ptr q,
					   std::vector<BinaryDescriptor::Ptr>* neigh,
					   std::vector<double>* distances,
					   unsigned knn = 2,
					   unsigned checks = 32)
	const;

	void insertDescriptor(BinaryDescriptor::Ptr q);
	void deleteDescriptor(BinaryDescriptor::Ptr q);
	void purgeDescriptors(const uint curr_img);

	template<class Archive>
	void save(Archive &ar, const unsigned int v) const;

	template<class Archive>
	void load(Archive &ar, const unsigned int v);

	BOOST_SERIALIZATION_SPLIT_MEMBER();

};

} /* namespace PlaceRecognizer */


#include "IncrementalBoW_serialization.h"


#endif /* PLACE_RECOGNIZER_INCREMENTALBOW_H_ */
