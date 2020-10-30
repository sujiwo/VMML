/*
 * bagviewer2.cpp
 *
 *  Created on: May 22, 2020
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <exception>
#include <QApplication>
#include <QComboBox>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "RandomAccessBag.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <GenericImagesetViewer.h>
#include <boost/filesystem.hpp>
#include "ui_GenericImagesetViewer.h"


using namespace std;
using namespace DsViewer;


class ImageBagDataset : public ImageDataset
{
public:
	friend class BagViewer2;

	void load (const string &bagpath)
	{
		bagFdPtr = std::shared_ptr<rosbag::Bag>(new rosbag::Bag(bagpath, rosbag::BagMode::Read));
		vTopicList = RandomAccessBag::getTopicList(*bagFdPtr);
		auto imgTopics = getImageTopics();
		setActiveTopic(imgTopics[0]);
		basename = boost::filesystem::basename(bagpath)+boost::filesystem::extension(bagpath);
	}

	void setActiveTopic(const string &tp)
	{
		currentActiveTopic = tp;
		bagSrc.reset(new RandomAccessBag(*bagFdPtr, tp));
	}

	vector<string> getImageTopics() const
	{
		vector<string> imgTopics;

		for (auto &stp: vTopicList) {
			if (stp.second=="sensor_msgs/Image" or
					stp.second == "sensor_msgs/CompressedImage")
				imgTopics.push_back(stp.first);
		}

		return imgTopics;
	}

	ImageDatasetObject at(uint i)
	{
		ImageDatasetObject igbo;
		igbo.id = i;
		igbo.timestamp = bagSrc->timeAt(i).toBoost();

		if (bagSrc->messageType() == "sensor_msgs/Image") {
			sensor_msgs::Image::ConstPtr imageMsg = bagSrc->at<sensor_msgs::Image>(i);
			igbo.image = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)->image;
		}
		else if (bagSrc->messageType() == "sensor_msgs/CompressedImage") {
			sensor_msgs::CompressedImage::ConstPtr imageMsg = bagSrc->at<sensor_msgs::CompressedImage>(i);
			igbo.image = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)->image;
		}

		return igbo;
	}

	virtual size_t size() const
	{ return bagSrc->size(); }

	virtual tduration length() const
	{ return bagSrc->length().toBoost(); }

	const string getName() const
	{ return basename; }

private:
	std::shared_ptr<rosbag::Bag> bagFdPtr = nullptr;
	RandomAccessBag::Ptr bagSrc;
	map<string,string> vTopicList;
	string basename;

	string currentActiveTopic;
};


class BagViewer2 : public GenericImagesetViewer
{
public:
	BagViewer2 (QWidget *parent = 0):
		GenericImagesetViewer(parent),
		topicSelector(new QComboBox)
	{
		ui->controlLayout->addWidget(topicSelector);
		this->connect(topicSelector,
			QOverload<int>::of(&QComboBox::currentIndexChanged),
			[&](int index){topicSelector_currentIndexChanged(index);});
	}

	void setDatasource(std::shared_ptr<ImageBagDataset> &ds)
	{
		this->setWindowTitle(QString::fromStdString(this->originalWindowTitle + " :: " + ds->getName()));

		realDataset = ds;
		imageTopics = realDataset->getImageTopics();
		shared_ptr<ImageDataset> imgSetPtr = static_pointer_cast<ImageDataset>(ds);
		GenericImagesetViewer::setDatasource(imgSetPtr);
		for (auto &tp: imageTopics) {
			topicSelector->addItem(QString(tp.c_str()));
		}
	}

	void topicSelector_currentIndexChanged(int i)
	{
		realDataset->setActiveTopic(imageTopics[i]);
		updateImage(0);
	}

private:
	vector<string> imageTopics;
	QComboBox *topicSelector;
	std::shared_ptr<ImageBagDataset> realDataset;
};


int main(int argc, char *argv[])
{
	QApplication mainApp(argc, argv);
	BagViewer2 viewer;

	shared_ptr<ImageBagDataset> imgSet(new ImageBagDataset);
	imgSet->load(argv[1]);
	shared_ptr<ImageDataset> imgSetPtr = static_pointer_cast<ImageDataset>(imgSet);
	viewer.setDatasource(imgSet);

	viewer.show();

	return mainApp.exec();
}
