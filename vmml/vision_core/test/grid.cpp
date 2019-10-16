/*
 * grid.cpp
 *
 *  Created on: Oct 16, 2019
 *      Author: sujiwo
 */


#include "utilities.h"


using namespace std;
using namespace Vmml;


class BucketInt : public Grid<vector<int>>
{
public:
	BucketInt(int width, int height, int depth)
	{
		mGrid.resize(height);
		for (int y=0; y<height; y++) {
			mGrid[y].resize(width);
			for (int x=0; x<width; x++) {
				mGrid[y][x].reset(new vector<int>(depth, 0));
			}
		}
	}

	int& operator()(const int x, const int y, const int z)
	{ return mGrid[y][x]->at(z); }

	const int& operator()(const int x, const int y, const int z) const
	{ return mGrid[y][x]->at(z); }

};



int main(int argc, char *argv[])
{
	return 0;
}
