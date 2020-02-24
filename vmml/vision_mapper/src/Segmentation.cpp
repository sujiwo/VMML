/*
 * Segmentation.cpp
 *
 *  Created on: Feb 22, 2020
 *      Author: sujiwo
 */

#include <Segmentation.h>


using namespace caffe;


namespace Vmml {
namespace Mapper {

Segmentation::Segmentation(const std::string &modelPath, const std::string &weights)
{
	Caffe::set_mode(Caffe::GPU);

	mNet.reset( new caffe::Net<float> (modelPath, caffe::TEST) );
	mNet->CopyTrainedLayersFrom(weights);
}


Segmentation::~Segmentation()
{}

} /* namespace Mapper */
} /* namespace Vmml */
