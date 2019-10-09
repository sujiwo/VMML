/*
 * Initializer.h
 *
 *  Created on: Oct 9, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_INITIALIZER_H_
#define VMML_CORE_INITIALIZER_H_

#include <opencv2/features2d.hpp>
#include "BaseFrame.h"


namespace Vmml {

class Initializer
{
public:
	Initializer(BaseFrame::Ptr initFrame);
	virtual ~Initializer();

	bool initialize(BaseFrame::Ptr frameNext);

protected:
	BaseFrame::Ptr frame0;
};

} /* namespace Vmml */

#endif /* VMML_CORE_INITIALIZER_H_ */
