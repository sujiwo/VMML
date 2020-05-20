/*
 * GenericDatasetViewer.h
 *
 *  Created on: May 20, 2020
 *      Author: sujiwo
 */

#ifndef _GENERICDATASETVIEWER_H_
#define _GENERICDATASETVIEWER_H_


#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/conversion.hpp>

#include <QMainWindow>
#include "rqt_image_view/ratio_layouted_frame.h"

namespace DsViewer {

typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::time_duration tduration;

struct ImageDataset {

	virtual void load(const std::string &path) = 0;

	virtual cv::Mat at(uint i);

	virtual size_t size() const;
	virtual tduration length() const;
};


class GenericDatasetViewer : public QMainWindow {
Q_OBJECT

public:
	explicit GenericDatasetViewer(QWidget *parent = 0);
	virtual ~GenericDatasetViewer();

	void setDatasource(std::shared_ptr<ImageDataset> &ds);

public slots:
	void on_playButton_clicked(bool checked);
	void on_playProgress_sliderMoved(int i);
	void on_saveButton_clicked(bool checked);
	void on_nextFrameBtn_clicked(bool checked);
	void on_prevFrameBtn_clicked(bool checked);
	void on_copyImageBtn_clicked(bool checked);
	void timeOffsetIndicator_clicked();

protected:

	std::shared_ptr<ImageDataset> dataSrc = nullptr;
};

}	// namespace DsViewer

#endif /* _GENERICDATASETVIEWER_H_ */
