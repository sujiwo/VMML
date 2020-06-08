/*
 * viewer.cpp
 *
 *  Created on: Jun 2, 2020
 *      Author: sujiwo
 */

#include <iostream>
#include <memory>
#include <QApplication>
#include "OxfordDataset.h"
#include <GenericImagesetViewer.h>


using namespace std;
using namespace oxf;


struct OxfordViewDataset : public DsViewer::ImageDataset
{
	virtual void load(const std::string &path)
	{
		rlDataObj.reset(new OxfordDataset(path));
	}

	virtual DsViewer::ImageDatasetObject at(uint i)
	{
		auto dsObj = rlDataObj->at(i);
		DsViewer::ImageDatasetObject viewObj;
		viewObj.id = i;
		cv::cvtColor(dsObj.center_image, viewObj.image, cv::COLOR_BGR2RGB);
		viewObj.timestamp = dsObj.timestamp;

		if (dsObj.center_image.empty()==true) {
			// dummy image
			viewObj.image = cv::Mat(rlDataObj->getCameraParameters().getImageSize(), CV_8UC3, cv::Scalar(0,0,0));
/*
			string text("Image not found");
			int baseline;
			auto txtsize = cv::getTextSize(text, cv::FONT_HERSHEY_COMPLEX, 1, 1, &baseline);
			cv::putText(viewObj.image,
				text,
				cv::Point((viewObj.image.cols-txtsize.width)/2, (viewObj.image.rows-txtsize.height)/2),
				cv::FONT_HERSHEY_COMPLEX,
				1,
				cv::Scalar(255,255,255), 1);
*/
		}

		return viewObj;
	}

	virtual size_t size() const
	{ return rlDataObj->size();	}

	virtual tduration length() const
	{ return rlDataObj->length(); }

protected:
	shared_ptr<OxfordDataset> rlDataObj;
};


class OxfordViewer : public DsViewer::GenericImagesetViewer
{
public:
	OxfordViewer(const string &path, QWidget *parent = 0):
		DsViewer::GenericImagesetViewer(parent),
		dataSrc(new OxfordViewDataset)
	{
		dataSrc->load(path);
		auto imgSetPtr = static_pointer_cast<DsViewer::ImageDataset>(dataSrc);
		setDatasource(imgSetPtr);
	}

protected:
	shared_ptr<OxfordViewDataset> dataSrc;
};


int main (int argc, char *argv[])
{
	QApplication mainApp(argc, argv);
	OxfordViewer viewer(argv[1]);

	viewer.show();

	return mainApp.exec();
}
