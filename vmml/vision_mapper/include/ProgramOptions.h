/*
 * ProgramOptions.h
 *
 *  Created on: Dec 26, 2019
 *      Author: sujiwo
 */

#ifndef VMML_MAPPER_PROGRAMOPTIONS_H_
#define VMML_MAPPER_PROGRAMOPTIONS_H_

#include <boost/program_options.hpp>


namespace Vmml {
namespace Mapper {

class ProgramOptions {
public:
	ProgramOptions(int argc, char *argv[]);
	virtual ~ProgramOptions();
};

} /* namespace Mapper */
} /* namespace Vmml */

#endif /* VMML_MAPPER_PROGRAMOPTIONS_H_ */
