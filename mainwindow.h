#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include "stereovisionblackbox.h"

using namespace cv;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_slider_maxDisp_valueChanged(int value);

	void on_slider_SADwindow_valueChanged(int value);

private:
	void showDisp();

	Ui::MainWindow *ui;
	StereoVisionBlackBox *svbox;
	Mat testL;
	Mat testR;
};

#endif // MAINWINDOW_H
