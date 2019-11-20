/*
 * KeyFrame.h
 *
 *  Created on: Oct 7, 2019
 *      Author: sujiwo
 */

#ifndef VMML_CORE_KEYFRAME_H_
#define VMML_CORE_KEYFRAME_H_

#include <boost/serialization/serialization.hpp>
#include <limits>
#include <memory>

#include "utilities.h"
#include "BaseFrame.h"


namespace Vmml { class KeyFrame; }
namespace boost {
namespace serialization {
	template <class Archive>
		void serialize (Archive & ar, Vmml::KeyFrame &keyframe, const unsigned int version);
}
}


namespace Vmml {


class VisionMap;


class KeyFrame : public BaseFrame
{
public:
	friend class VisionMap;

	typedef std::shared_ptr<KeyFrame> Ptr;

	KeyFrame(const std::shared_ptr<VisionMap> _parent);

	KeyFrame(cv::Mat img,
		const std::shared_ptr<VisionMap> _parent,
		int cameraNo=0,
		const ptime &ts=MAX_TIME,
		const Pose &p=Pose::Identity(),
		bool doComputeFeatures=true
		);

	KeyFrame(const BaseFrame &bsFrame, const std::shared_ptr<VisionMap> _parent, int cameraNo=0, const ptime &ts=MAX_TIME);

	virtual ~KeyFrame();

	const std::shared_ptr<VisionMap> parent() const
	{ return mParent; }

	std::shared_ptr<VisionMap> parent()
	{ return mParent; }

	// XXX: Define more concrete constructor

	inline kfid getId() const
	{ return id; }

	static Ptr
	create(cv::Mat image, const std::shared_ptr<VisionMap>& mParent, int cameraNumber=0, const ptime &ts=MAX_TIME);

	static Ptr
	fromBaseFrame(const BaseFrame &frameSource, const std::shared_ptr<VisionMap>& mParent, int cameraNumber=0, const ptime &ts=MAX_TIME);

	void computeBoW();

	static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

	double computeSceneMedianDepth() const;

protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, Vmml::KeyFrame &keyframe, const unsigned int version);

	kfid id;
	int cameraId;
	sourceId srcId=0;

	// Time at which the image was taken
	ptime frCreationTime;

	static kfid nextId;

	const std::shared_ptr<VisionMap> mParent;
};

} /* namespace Vmml */

#endif /* VMML_CORE_KEYFRAME_H_ */
