/*
 * test_program_options.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */


#include "ProgramOptions.h"


using namespace Vmml;
using namespace Vmml::Mapper;
namespace po=boost::program_options;
using po::value;

ProgramOptions nOpts;

int main(int argc, char *argv[])
{
	int startFrame;
	nOpts.addOptions()("start",
		value<int>()->default_value(0)->notifier(
		[&](const int &i){startFrame=i;}),
		"Starting frame for images, numbered from 0");

	nOpts.parseCommandLineArgs(argc, argv);
	return 0;
}
