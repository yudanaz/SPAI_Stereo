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
//	vector<Mat> calibImgs1;
//	vector<Mat> calibImgs2;
//	QDir calibDir("/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/stereoCalibImages");
//	QStringList files = calibDir.entryList();
//	foreach(QString file, files)
//	{
//		QString url = calibDir.absoluteFilePath(file);
//		qDebug() << url;
//		Mat m = imread(url.toStdString(), IMREAD_GRAYSCALE);
//		if(file.contains("Left")){ calibImgs1.push_back(m); }
//		if(file.contains("Right")){ calibImgs2.push_back(m); }
//	}
//	svbox = new StereoVisionBlackBox( ((Mat)calibImgs1[0]).rows, ((Mat)calibImgs1[0]).cols);
//	qDebug() << "adding calibration images for stereo camera rig: "
//			 << svbox->addCalibrationImages(calibImgs1, calibImgs2, CALIB_IMGS_STEREO_CAMS);

	svbox = new StereoVisionBlackBox(480, 640);
	qDebug() << "adding calibration images for stereo camera rig: "
			 << svbox->addCalibrationImages("/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/Calibration/Left_jpg",
								"/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/Calibration/Right_jpg", CALIB_IMGS_STEREO_CAMS);

	//try out the stereo vision blackbox
	qDebug() << "calibrating stereo cameras: "
			 << svbox->calibrateCameras(Size(12,12));

	qDebug() << "saving calibration file: " << svbox->saveCalibration("calibfile.clb");
	qDebug() << "loading calibration file: " << svbox->loadCalibration("calibfile.clb");


//	testL = calibImgs1[4];
//	testR = calibImgs2[4];
	testL = imread("/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/Calibration/Left_jpg/Left15.jpg", IMREAD_GRAYSCALE);
	testR = imread("/home/maurice/Qt_Projects/build-SPAI_StereoDev-Desktop_Qt_5_2_1_GCC_64bit-Debug/Calibration/Right_jpg/Right15.jpg", IMREAD_GRAYSCALE);
	if(svbox->isCalibrated())
	{
		showDisp();
	}
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::showDisp()
{
	Mat rectL, rectR;
	Mat disp = svbox->getDisparityMap(testL, testR, &rectL, &rectR);
	Mat disp8bit;
	double min, max;
	minMaxLoc(disp, &min, &max);
	disp.convertTo(disp8bit, CV_8UC1, 255.0 / max);
//	imshow("original rectified left image", rectR);
//	imshow("original rectified right image", rectR);
	imshow("disparity image", disp8bit);
}

void MainWindow::on_slider_maxDisp_valueChanged(int value)
{
	if(!svbox->isCalibrated()){ return; }
	svbox->setStereoMatchingParameters(value);
	qDebug() << " max disp: " << value;
	showDisp();
}

void MainWindow::on_slider_SADwindow_valueChanged(int value)
{
	if(!svbox->isCalibrated()){ return; }
	svbox->setStereoMatchingParameters(-1, value);
	qDebug() << " SAD windows: " << value;
	showDisp();
}
