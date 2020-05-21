/*
 * GenericDatasetViewer.cpp
 *
 *  Created on: May 20, 2020
 *      Author: sujiwo
 */

#include "GenericImagesetViewer.h"
#include "ui_GenericImagesetViewer.h"


namespace DsViewer {


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
}

GenericImagesetViewer::~GenericImagesetViewer()
{}

void GenericImagesetViewer::disableControlsOnPlaying(bool state)
{
	ui->saveButton->setDisabled(state);
	ui->playProgress->setDisabled(state);
}


}	// namespace DsViewer


