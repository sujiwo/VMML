/*
 * FFrame.h
 *
 *  Created on: Oct 10, 2019
 *      Author: sujiwo
 */

/*
 * BaseFrame with computed BoW vectors
 */

#ifndef VMML_CORE_FFRAME_H_
#define VMML_CORE_FFRAME_H_

#include "BaseFrame.h"
#include "ORBVocabulary.h"


namespace Vmml
{

class FFrame : public BaseFrame
{
public:

	typedef std::shared_ptr<FFrame> Ptr;

	FFrame();
	virtual ~FFrame();

	static Ptr
	fromBaseFrame(const BaseFrame& srcFrame, const ORBVocabulary &voc);

protected:
	DBoW2::BowVector mBowVec;
	DBoW2::FeatureVector mFeatVec;

};

} /* namespace Vmml */

#endif /* VMML_CORE_FFRAME_H_ */
