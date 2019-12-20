/*
 * ORBVocabulary.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_ORBVOCABULARY_H_
#define VMML_CORE_ORBVOCABULARY_H_

#include <vector>
#include <set>
#include <map>
#include <unistd.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include "DBoW2/BowVector.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "cvobj_serialization.h"


namespace Vmml {

class ORBVocabulary : public DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
{
protected:
	friend class boost::serialization::access;

	bool vocabularyLoaded = false;

	template<class Archive>
	void save(Archive &ar, const unsigned int v) const
	{
		ar << m_k;
		ar << m_L;
		ar << m_scoring;
		ar << m_weighting;

		uint numNodes = m_nodes.size();
		uint numWords = m_words.size();
		ar << numNodes;
		ar << numWords;

		for (int i=0; i<numNodes; i++) {
			const Node &n = m_nodes[i];
			ar << n.id;
			ar << n.id;
			ar << n.weight;
			ar << n.children;
			ar << n.parent;
			ar << n.descriptor;
			ar << n.word_id;
		}
	}

	template<class Archive>
	void load(Archive &ar, const unsigned int v)
	{
		ar >> m_k;
		ar >> m_L;
		ar >> m_scoring;
		ar >> m_weighting;

		createScoringObject();

		uint numNodes, numWords;
		ar >> numNodes;
		ar >> numWords;

		// nodes
	    m_nodes.resize(numNodes);
	    m_words.resize(numWords);

	    for (int i=0; i<numNodes; i++) {
	    	Node &n = m_nodes[i];
	    	ar >> n.id;
			ar >> n.id;
			ar >> n.weight;
			ar >> n.children;
			ar >> n.parent;
			ar >> n.descriptor;
			ar >> n.word_id;

			if (n.isLeaf()) {
				m_words[n.word_id] = &n;
			}
	    }
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER();

public:

	inline bool loadFromTextFile(const string &filename)
	{
		assert (access(filename.c_str(), R_OK)==0);
		m_nodes.clear();
		m_words.clear();
		return DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>::loadFromTextFile(filename);
	}


	// Call these function to build vocabulary
	void prepareTrain();
	void train(const std::vector<DBoW2::FORB::TDescriptor> &frameDescriptor);
	void build();

protected:
	// XXX: put a lock for multi-threaded vocabulary building
	std::vector<DBoW2::FORB::TDescriptor> kpTrains;

	void HKmeansStep(
		DBoW2::NodeId parent_id,
		const vector<pDescriptor> &descriptors,
		int current_level);
};


} /* namespace Vmml */

#endif /* VMML_CORE_ORBVOCABULARY_H_ */
