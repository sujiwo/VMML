/*
 * ndt_odom.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: sujiwo
 */


#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include "pcl/registration/ndt.h"
#include "pcl/filters/voxel_grid.h"
#include "vmml/LidarScanBag.h"
#include "vmml/Trajectory.h"
#include "vmml/utilities.h"

using namespace std;
using namespace Vmml;


struct MessageDivision {
	uint from, to;
};


void createIntervals(const uint numOfMsg, vector<MessageDivision> &percpu)
{
	const uint numOfCpus = thread::hardware_concurrency(),
		len = numOfMsg / numOfCpus;
	percpu.clear();

	for (int i=0; i<numOfCpus; i++) {
		MessageDivision md;
		md.from = i*len;
		md.to = (i==numOfCpus-1) ? numOfMsg : md.from+len;
		percpu.push_back(md);
	}
}


class NdtOdometry
{
public:
	typedef pcl::PointXYZI Point3;
	typedef pcl::PointCloud<Point3> Cloud3;


	NdtOdometry(rosbag::Bag &bagSrc, const std::string &velodyneTopic) :
		pcdScans(bagSrc, velodyneTopic)
	{}


	void runSingle()
	{
		Cloud3::ConstPtr anchor = nullptr;
		Pose lastPose = Pose::Identity();
		ptime timestamp;
		pcl::NormalDistributionsTransform<NdtOdometry::Point3, NdtOdometry::Point3> mNdt;

		const uint maxNum = pcdScans.size();
		for (int i=0; i<maxNum; ++i) {
			auto scan = pcdScans.getFiltered<NdtOdometry::Point3>(i, &timestamp);
			if (anchor==nullptr) {
				anchor = pcdScans.getUnfiltered<NdtOdometry::Point3>(i);
				bagTrack.push_back(PoseStamped(lastPose, timestamp));
				continue;
			}

			mNdt.setInputTarget(anchor);
			mNdt.setInputSource(scan);
			NdtOdometry::Cloud3 pOut;
			ptime t1 = getCurrentTime();
			mNdt.align(pOut);
			ptime t2 = getCurrentTime();

			TTransform currentTrans = mNdt.getFinalTransformation().cast<double>();
			lastPose = lastPose * currentTrans;
			bagTrack.push_back(PoseStamped(lastPose, timestamp));

			auto newAnchor = pcdScans.getUnfiltered<NdtOdometry::Point3>(i);
			anchor = newAnchor;

			cout << i+1 << " / " << maxNum << "; " << toSeconds(t2-t1) << endl;
		}
	}


	struct NdtResultPerChild {
		uint cpuId=0;
		TTransform firstScan=TTransform::Identity();
		Trajectory matchResult;
		vector<tduration> scanDurations;
	};


	void runMulti()
	{
		vector<MessageDivision> cpuDivs;
		createIntervals(pcdScans.size(), cpuDivs);

		const uint numOfCpus = std::thread::hardware_concurrency();
		vector<NdtResultPerChild> childResults(numOfCpus);
		std::vector<std::thread> childs(numOfCpus);
		mutex ioMtx;

		for (int c=0; c<numOfCpus; c++) {
			childs[c] = std::thread([c, &childResults, &cpuDivs, &ioMtx, this] {

				Cloud3::ConstPtr anchor = nullptr;
				Pose lastPose = Pose::Identity();
				pcl::NormalDistributionsTransform<NdtOdometry::Point3, NdtOdometry::Point3> mNdt;

				MessageDivision &myjob = cpuDivs[c];
				auto &mywork = childResults[c];
				mywork.cpuId = c;

				for (int i=myjob.from, p=0; i<myjob.to; i++, p++) {

					ptime timestamp;
					// Bag reading is not thread-safe
					Cloud3::ConstPtr scan = this->readLidarScanWithLock(i, true, &timestamp);

					if (anchor==nullptr) {
						anchor = this->readLidarScanWithLock(i, false);
						mywork.matchResult.push_back(PoseStamped(lastPose, timestamp));
						continue;
					}

					mNdt.setInputTarget(anchor);
					mNdt.setInputSource(scan);
					NdtOdometry::Cloud3 pOut;
					ptime t1 = getCurrentTime();
					mNdt.align(pOut);
					ptime t2 = getCurrentTime();

					TTransform currentTrans = mNdt.getFinalTransformation().cast<double>();
					lastPose = lastPose * currentTrans;
					mywork.matchResult.push_back(PoseStamped(lastPose, timestamp));
					mywork.scanDurations.push_back(t2-t1);

					auto newAnchor = this->readLidarScanWithLock(i, false);
					anchor = newAnchor;

					{
						lock_guard<mutex> screenLock(ioMtx);
						cout << c << ": " << p << " / " << myjob.to-myjob.from << "; " << toSeconds(mywork.scanDurations.back()) << endl;
					}
				}

				mywork.matchResult.dump("test-mt"+to_string(c)+".csv");
			});
		}

		for (auto &t: childs) {
			t.join();
		}

		// Assemble the result
		TTransform lastRigidT = TTransform::Identity();
		bagTrack.clear();
		for (int c=0; c<numOfCpus; c++) {
			for (int p=0; p<childResults[c].matchResult.size(); p++) {
				PoseStamped pv;
				if (p==0) {
					pv = childResults[c].matchResult[0] * lastRigidT;
				}
				else {
					TTransform tv = childResults[c].matchResult[p-1].inverse() * childResults[c].matchResult[p];
					pv = PoseStamped(bagTrack.back() * tv, childResults[c].matchResult[p].timestamp);
				}
				bagTrack.push_back(pv);
			}
			lastRigidT = bagTrack.back();
		}
	}


	const Trajectory& getTrajectory() const
	{ return bagTrack; }


	Cloud3::ConstPtr readLidarScanWithLock(const uint p, bool filtered, ptime *timestamp=NULL)
	{
		lock_guard<mutex> bgl(bagLock);
		if (filtered)
			return pcdScans.getFiltered<Point3>(p, timestamp);
		else
			return pcdScans.getUnfiltered<Point3>(p, timestamp);
	}


protected:
	LidarScanBag pcdScans;
	Trajectory bagTrack;
	mutex bagLock;
};



int main(int argc, char *argv[])
{
	rosbag::Bag inputBag(argv[1]);

	NdtOdometry odom(inputBag, "/velodyne_packets");
	odom.runMulti();

	odom.getTrajectory().dump("test.csv");
	return 0;
}
