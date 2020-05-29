/*
 * oxford_open.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include "OxfordDataset.h"

using namespace std;
using namespace oxf;

int main(int argc, char *argv[])
{
	OxfordDataset dataset(argv[1]);

	auto dataMid = dataset.at(dataset.size()/2);
	cv::imwrite("/tmp/2.png", dataMid.center_image);

	return 0;
}
