/*
 * mod.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include "im_enhance.h"
#include <pybind11/pybind11.h>


void im_test()
{
	std::cout << "Hello python\n";
}


namespace py = pybind11;
PYBIND11_PLUGIN(ime_py) {
	py::module m("ime_py", "Image contrast enhancement routine");

	m.def("test", &im_test, "Validate imtest");

	return m.ptr();
}
