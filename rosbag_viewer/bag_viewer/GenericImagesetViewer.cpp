/*
 * GenericDatasetViewer.cpp
 *
 *  Created on: May 20, 2020
 *      Author: sujiwo
 */

#include <GenericImagesetViewer.h>
#include <thread>
#include <iostream>
#include <QString>
#include <QFileDialog>
#include <QClipboard>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ui_GenericImagesetViewer.h"


using namespace std;


namespace DsViewer {


double toSeconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }


inline ptime getCurrentTime() {
  return boost::posix_time::microsec_clock::local_time();
}


GenericImagesetViewer::GenericImagesetViewer(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::GenericImagesetViewer)
{
	ui->setupUi(this);

	timeOffsetIndicator = new ClickableLabel("0.00", this);
	ui->statusbar->addPermanentWidget(timeOffsetIndicator);
	this->connect(timeOffsetIndicator, SIGNAL(clicked()), this,
			SLOT(timeOffsetIndicator_clicked()));
	ui->imagePlace->setOuterLayout(ui->imageLayout);

	originalWindowTitle = this->windowTitle().toStdString();
}


GenericImagesetViewer::~GenericImagesetViewer()
{}


void
GenericImagesetViewer::disableControlsOnPlaying(bool state)
{
	ui->saveButton->setDisabled(state);
	ui->playProgress->setDisabled(state);
}


void
GenericImagesetViewer::updateImage(int n)
{
	if (n >= dataSrc->size())
		return;
	currentPosition = n;

	auto imageObject = dataSrc->at(n);

	currentImage = imageObject.image;
	const QImage curImage(currentImage.data, currentImage.cols, currentImage.rows,
			currentImage.step[0], QImage::Format_RGB888);
	ui->imagePlace->setImage(curImage);
	updateTimeOffsetIndicator();
}


void
GenericImagesetViewer::on_saveButton_clicked(bool checked)
{
	string fname = QFileDialog::getSaveFileName(this, tr("Save Image")).toStdString();
	if (fname.length() == 0)
		return;

	if (fname.substr(fname.size(), 4)==".mat") {

	}

	else {
		cv::Mat rgb;
		cv::cvtColor(currentImage, rgb, cv::COLOR_RGB2BGR);

		cv::imwrite(fname, rgb);
	}
}


void
GenericImagesetViewer::on_copyImageBtn_clicked(bool checked)
{
	QClipboard *clipboard = QGuiApplication::clipboard();
	const QImage curImage(currentImage.data, currentImage.cols, currentImage.rows,
		currentImage.step[0], QImage::Format_RGB888);
	clipboard->setImage(curImage);
}


void
GenericImagesetViewer::timeOffsetIndicator_clicked()
{
	if (timeOffsetIndicatorMode == OFFSET_TIME)
		timeOffsetIndicatorMode = OFFSET_INTEGER;
	else if (timeOffsetIndicatorMode == OFFSET_INTEGER)
		timeOffsetIndicatorMode = OFFSET_TIME;

	updateTimeOffsetIndicator();
}


void
GenericImagesetViewer::setDatasource(std::shared_ptr<ImageDataset> &ds)
{
	dataSrc = ds;
	ui->playProgress->setRange(0, dataSrc->size()-1);
	ui->playProgress->setValue(0);
	updateImage(0);
}


void
GenericImagesetViewer::on_playProgress_sliderMoved(int i)
{ updateImage(i); }


void
GenericImagesetViewer::on_nextFrameBtn_clicked(bool checked)
{ updateImage(currentPosition+1); }


void
GenericImagesetViewer::on_prevFrameBtn_clicked(bool checked)
{
	if (currentPosition==0) return;
	updateImage(currentPosition-1);
}


void GenericImagesetViewer::on_playButton_clicked(bool checked)
{
	static bool playStarted = false;
	static std::thread *playerThread = NULL;

	std::function<void()> playThreadFn = [&]() {
		const int startPos = ui->playProgress->sliderPosition();
		disableControlsOnPlaying(true);
		for (int p = startPos; p <= ui->playProgress->maximum(); p++) {

			ptime t1x = getCurrentTime();
			ui->playProgress->setSliderPosition(p);
			updateImage(p);
			if (playStarted == false)
				break;

			if (p < ui->playProgress->maximum()) {
				ptime t1 = dataSrc->at(p).timestamp,
						t2 = dataSrc->at(p + 1).timestamp;
				ptime t2x = getCurrentTime();
				tduration tdx = t2x - t1x; // processing overhead
				tduration td = (t2 - t1) - tdx;
				std::this_thread::sleep_for(
					std::chrono::milliseconds(td.total_milliseconds()));
			}
		}
		disableControlsOnPlaying(false);
	};

	if (checked == true) {
		playStarted = true;
		playerThread = new std::thread(playThreadFn);
	}

	else {
		playStarted = false;
		playerThread->join();
		delete (playerThread);
	}

	return;
}


void GenericImagesetViewer::updateTimeOffsetIndicator()
{
	string toi;
	auto td = dataSrc->at(currentPosition).timestamp;

	if (timeOffsetIndicatorMode == OFFSET_TIME) {
		auto tx = td - dataSrc->at(0).timestamp;
		stringstream ss;
		ss << fixed << setprecision(2) << toSeconds(tx);
		toi = ss.str();
	}

	else if (timeOffsetIndicatorMode == OFFSET_INTEGER) {
		toi = "Image: ";
		toi += std::to_string(currentPosition);
	}

	timeOffsetIndicator->setText(QString::fromStdString(toi));
}


}	// namespace DsViewer


