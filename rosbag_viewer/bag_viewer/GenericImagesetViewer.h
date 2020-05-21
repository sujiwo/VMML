/*
 * GenericDatasetViewer.h
 *
 *  Created on: May 20, 2020
 *      Author: sujiwo
 */

#ifndef _GENERICIMAGESETVIEWER_H_
#define _GENERICIMAGESETVIEWER_H_


#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/conversion.hpp>

#include "ClickableLabel.h"
#include <QMainWindow>
#include "ratio_layouted_frame.h"

namespace Ui {
class GenericImagesetViewer;
}


namespace DsViewer {

typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::time_duration tduration;

struct ImageDatasetObject {
	uint id=0;
	virtual cv::Mat image() { return cv::Mat();};
	ptime timestamp;
};

struct ImageDataset {

	virtual void load(const std::string &path) = 0;

	virtual ImageDatasetObject at(uint i)=0;

	virtual size_t size() const;
	virtual tduration length() const;
};


class GenericImagesetViewer : public QMainWindow {
Q_OBJECT

public:
	explicit GenericImagesetViewer(QWidget *parent = 0);
	virtual ~GenericImagesetViewer();

	void setDatasource(std::shared_ptr<ImageDataset> &ds);

public slots:
	void on_playButton_clicked(bool checked);
	void on_playProgress_sliderMoved(int i);
	void on_saveButton_clicked(bool checked);
	void on_nextFrameBtn_clicked(bool checked);
	void on_prevFrameBtn_clicked(bool checked);
	void on_copyImageBtn_clicked(bool checked);
	void timeOffsetIndicator_clicked();

private:
  Ui::GenericImagesetViewer *ui;

protected:

	std::shared_ptr<ImageDataset> dataSrc = nullptr;

	virtual void updateImage(int n);

	void disableControlsOnPlaying(bool state);

	// Identifies current playing position in integer
	int currentPosition=0;

	ClickableLabel *timeOffsetIndicator;
	enum { OFFSET_INTEGER, OFFSET_TIME } timeOffsetIndicatorMode = OFFSET_TIME;
	void updateTimeOffsetIndicator();
};

}	// namespace DsViewer

#endif /* _GENERICIMAGESETVIEWER_H_ */
