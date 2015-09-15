#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "stereovisionblackbox.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;

    //try out the stereo vision blackbox
    StereoVisionBlackBox svbox(320,240);
}
