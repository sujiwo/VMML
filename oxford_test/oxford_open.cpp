/*
 * oxford_open.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <rosbag/bag.h>
#include "OxfordDataset.h"

using namespace std;
using namespace oxf;


int main(int argc, char *argv[])
{
	OxfordDataset dataset(argv[1]);

	auto chk = dataset.checkImages();
	int invalidImages = 0;
	for (auto c: chk) {
		if (c==false)
			invalidImages++;
	}
	cout << invalidImages << " invalid images out of " << dataset.size() << endl;

	return 0;
}
