/*
 * test_program_options.cpp
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */


#include "ProgramOptions.h"


using namespace Vmml;
using namespace Vmml::Mapper;


ProgramOptions nOpts;

int main(int argc, char *argv[])
{
	nOpts.parseCommandLineArgs(argc, argv);
	return 0;
}
