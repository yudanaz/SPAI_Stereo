#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QStringList>
#include <QDir>
#include <QDebug>
#include "stereovisionblackbox.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	//get calib files from disk
	vector<Mat> calibImgs1;
	vector<Mat> calibImgs2;
	QDir calibDir("/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/stereoCalibImages");
	QStringList files = calibDir.entryList();
	foreach(QString file, files)
	{
		QString url = calibDir.absoluteFilePath(file);
		qDebug() << url;
		Mat m = imread(url.toStdString(), IMREAD_GRAYSCALE);
		if(file.contains("Left")){ calibImgs1.push_back(m); }
		if(file.contains("Right")){ calibImgs2.push_back(m); }
	}

	//try out the stereo vision blackbox
	StereoVisionBlackBox svbox( ((Mat)calibImgs1[0]).rows, ((Mat)calibImgs1[0]).cols);
	qDebug() << "adding calibration images: "
			 << svbox.addCalibrationImages(calibImgs1, calibImgs2, Size(12,12));
	qDebug() << "calibrating stereo cameras: "
			 << svbox.calibrateCameras(true);
}

MainWindow::~MainWindow()
{
	delete ui;
}
