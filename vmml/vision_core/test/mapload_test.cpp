/*
 * mapload_test.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: sujiwo
 */

#include "VisionMap.h"


int main(int argc, char *argv[])
{
	Vmml::VisionMap::Ptr vMap = Vmml::VisionMap::create();
	vMap->load("/tmp/maptest.vmap");

	return 0;
}
