/*
 * MapBuilder.cpp
 *
 *  Created on: Oct 8, 2019
 *      Author: sujiwo
 */

#include <MapBuilder.h>


using namespace std;


namespace Vmml {


MapBuilder::MapBuilder(const CameraPinholeParams &camera0, float zoom) :
	zoomRatio(zoom)
{
	vMap.reset(new VisionMap);
	auto cameraZ = camera0 * zoom;
	vMap->addCameraParameter(cameraZ);
}


/*
void
MapBuilder::build(const RandomAccessBag &imageBag,
	sourceId startNum,
	sourceId stopNum)
{
	if (stopNum==numeric_limits<sourceId>::max())
		stopNum = imageBag.size()-1;

	assert(startNum < imageBag.size()-1);

	for (uint64_t p=startNum; p<=stopNum; p++) {

	}
}


KeyFrame::Ptr
MapBuilder::createKeyFrame(const RandomAccessBag &imageBag, sourceId n) const
{

}
*/
bool
MapBuilder::feed(cv::Mat inputImage)
{

}


MapBuilder::~MapBuilder() {
	// TODO Auto-generated destructor stub
}

} /* namespace Vmml */
