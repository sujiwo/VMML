/*
 * VisualOdometry.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: sujiwo
 */

#include <VisualOdometry.h>

namespace Vmml {

VisualOdometry::VisualOdometry(Parameters par) :
	param(par),
	featureGrid(par.bucket_width, par.bucket_height)
{
	// TODO Auto-generated constructor stub

}

VisualOdometry::~VisualOdometry() {
	// TODO Auto-generated destructor stub
}

} /* namespace Vmml */
