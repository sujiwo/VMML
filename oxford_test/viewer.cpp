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
		viewObj.image = dsObj.center_image;
		viewObj.timestamp = dsObj.timestamp;
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
