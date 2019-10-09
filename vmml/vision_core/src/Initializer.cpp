/*
 * Initializer.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: sujiwo
 */

#include "Initializer.h"

using namespace std;


namespace Vmml {

Initializer::Initializer(BaseFrame::Ptr initFrame, float maxDistanceInPixel, int iters) :
	frame0(initFrame),
	sigma(maxDistanceInPixel),
	maxIterations(iters)
{
	// TODO Auto-generated constructor stub

}


Initializer::~Initializer()
{
	// TODO Auto-generated destructor stub
}


bool Initializer::initialize(BaseFrame::Ptr frameNext)
{

}

} /* namespace Vmml */
