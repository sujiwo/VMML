/*
 * OxfordDataset.cpp
 *
 *  Created on: May 25, 2020
 *      Author: sujiwo
 */

#include "csv.h"
#include "OxfordDataset.h"


using namespace std;


namespace oxf {


/*
 * XXX: Oxford Timestamp is in Microsecond
 */

ptime fromOxfordTimestamp (const uint64_t ot)
{
	static const ptime epoch(boost::gregorian::date(1970, 1, 1));
	return epoch + boost::posix_time::microseconds(ot);
}


uint64_t toOxfordTimestamp (const ptime &t)
{
	static const ptime epoch(boost::gregorian::date(1970, 1, 1));
	auto d = t-epoch;
	return d.total_microseconds();
}


OxfordDataset::OxfordDataset(const std::string &path) :
	dirpath(path)
{
	loadTimestamps();
	loadModel();
}


OxfordDataset::~OxfordDataset() {
	// TODO Auto-generated destructor stub
}


void
OxfordDataset::loadTimestamps()
{
	const string timestampsPath = (dirpath / "stereo.timestamps").string();
	StringTable TS = create_table(timestampsPath);
	// It is likely that the last timestamp does not have related image files
	const size_t ss = TS.size()-1;
	stereoTimestamps.resize(ss);

	for (uint32_t i=0; i<ss; i++) {
		const string &tsstr = TS.get(i,0);
		const uint64 ts = stoul(tsstr);
		stereoTimestamps[i] = fromOxfordTimestamp(ts);
	}
}


void
OxfordDataset::loadModel()
{

}

} /* namespace oxf */


