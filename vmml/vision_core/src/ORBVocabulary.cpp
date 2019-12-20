/*
 * ORBVocabulary.cpp
 *
 *  Created on: Dec 11, 2019
 *      Author: sujiwo
 */

#include "vmml/ORBVocabulary.h"


using namespace std;


namespace Vmml {


void ORBVocabulary::prepareTrain()
{
	m_nodes.clear();
	m_words.clear();

	// expected_nodes = Sum_{i=0..L} ( k^i )
	int expected_nodes =
		(int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));

	m_nodes.reserve(expected_nodes); // avoid allocations when creating the tree
	kpTrains.clear();
}


void
ORBVocabulary::train(const std::vector<DBoW2::FORB::TDescriptor> &frameDescriptor)
{
	const uint startPut = kpTrains.size();
	kpTrains.reserve(startPut + frameDescriptor.size());

	for (uint i=startPut, c=0; i<startPut + frameDescriptor.size(); ++i, ++c) {
		if (frameDescriptor[c].empty()) {
			cerr << "Found an empty descriptor\n";
			abort();
		}
		kpTrains[i] = frameDescriptor[c].clone();
	}
}


void
ORBVocabulary::build()
{
	  // create root
	  m_nodes.push_back(Node(0)); // root

	  // create the tree
//	  HKmeansStep(0, features, 1);

	  // create the words
	  createWords();

	  // and set the weight of each node of the tree
//	  setNodeWeights(training_features);
}

}	// namespace Vmml
