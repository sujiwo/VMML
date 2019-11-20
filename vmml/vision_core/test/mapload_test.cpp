/*
 * mapload_test.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: sujiwo
 */

#include "VisionMap.h"


int main(int argc, char *argv[])
{
	Vmml::VisionMap vMap;
	vMap.load("/tmp/maptest.vmap");

	return 0;
}
