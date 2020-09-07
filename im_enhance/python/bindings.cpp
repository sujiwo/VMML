/*
 * mod.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: sujiwo
 */

#include <Python.h>
#include <iostream>
#include "conversion.h"
#include "im_enhance.h"


// https://realpython.com/build-python-c-extension-module/

void im_test()
{
	std::cout << "Hello python\n";
}


static PyObject *method_im_test(PyObject *self, PyObject *args)
{
//	return Py
	std::cout << "ICE printout\n";
	return NULL;
}


static PyObject *method_autoAdjustGammaRGB(PyObject *self, PyObject *args)
{
	NDArrayConverter cvt;

}


static PyObject *method_multiScaleRetinexCP(PyObject *self, PyObject *args)
{

}


static PyObject *method_dynamicHistogramEqualization(PyObject *self, PyObject *args)
{

}


static PyObject *method_exposureFusion(PyObject *self, PyObject *args)
{

}


static PyMethodDef IceMethods[] = {
    {"im_test", method_im_test, METH_NOARGS, "Test Method"},
	{"autoAdjustGammaRGB", method_autoAdjustGammaRGB, METH_VARARGS, "Automatic gamma adjusment"},
	{"multiScaleRetinexCP", method_multiScaleRetinexCP, METH_VARARGS, "Multi-scale Retinex with Color Preservation"},
	{"dynamicHistogramEqualization", method_dynamicHistogramEqualization, METH_VARARGS, "Dynamic Histogram Equalization"},
	{"exposureFusion", method_exposureFusion, METH_VARARGS, "Exposure Fusion"},
    {NULL, NULL, 0, NULL}
};


PyMODINIT_FUNC
initice(void)
{
	(void) Py_InitModule("ice", IceMethods);
}
