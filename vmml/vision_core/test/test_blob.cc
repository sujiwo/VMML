/*
 * test_blob.cc
 *
 *  Created on: Dec 23, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace std;


struct Blob
{
	typedef shared_ptr<Blob> Ptr;

	uint A;
	string name;

	template<class Archive>
	void serialize(Archive &a, unsigned int v)
	{
		a & A & name;
	}
};


int main(int argc, char *argv[])
{
	Blob::Ptr XP(new Blob);
	XP->A = 1;
	XP->name = "whoami";

	fstream outfd;
	outfd.open("/tmp/test.data", fstream::out | fstream::trunc);
	boost::archive::binary_oarchive outArc(outfd);

	outArc << XP;
	outfd.close();

	fstream inpfd;
	inpfd.open("/tmp/test.data", fstream::in);
	boost::archive::binary_iarchive inpArc(inpfd);
	Blob::Ptr Xin;
	inpArc >> Xin;
	inpfd.close();

	return 0;
}
