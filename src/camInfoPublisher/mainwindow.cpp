#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <chrono>
#include <thread>

#include <QFileDialog>
#include <QMessageBox>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

void MainWindow::slotPushButtonOpenFile()
{
    //Read the caminfo file
    QString dir = QFileDialog::getOpenFileName(
        this, 
        tr("Open File"), 
        "", 
        tr("Camera Info (*.txt *.yaml)")
    );
    
    if (dir.isEmpty())
    {
	    QMessageBox msgBox;
	    msgBox.setText("No custom directory was selected.");
	    msgBox.exec();
	    return;
    }
    
    std::string stddir = dir.toStdString();
    std::string camname;
    sensor_msgs::CameraInfo infoBuf;
    
    //parse the file
    bool success = camera_calibration_parsers::readCalibration(stddir, camname, infoBuf);
    
    QString notification;
    if(success)
    {
        notification = "parse succeeded";
    }
    else
    {
        notification = "parse failed";
    }
    
    QMessageBox msgBox;
    msgBox.setText(notification);
    msgBox.exec();
    
    if(!success)
    {
        return;
    }
    
    // Info retrieved!
    info = infoBuf;
    
    //Visualize content of the file
    ui->labelHeight->setText(QString::number(info.height));
    ui->labelWidth->setText(QString::number(info.width));
    
    ui->labelDisMod->setText(QString::fromStdString(info.distortion_model));
    
    ui->labelDisParam0->setText(QString::number(info.D[0]));
    ui->labelDisParam1->setText(QString::number(info.D[1]));
    ui->labelDisParam2->setText(QString::number(info.D[2]));
    ui->labelDisParam3->setText(QString::number(info.D[3]));
    ui->labelDisParam4->setText(QString::number(info.D[4]));
    
    ui->labelMatDis00->setText(QString::number(info.K[0]));
    ui->labelMatDis01->setText(QString::number(info.K[1]));
    ui->labelMatDis02->setText(QString::number(info.K[2]));
    
    ui->labelMatDis10->setText(QString::number(info.K[3]));
    ui->labelMatDis11->setText(QString::number(info.K[4]));
    ui->labelMatDis12->setText(QString::number(info.K[5]));
    
    ui->labelMatDis20->setText(QString::number(info.K[6]));
    ui->labelMatDis21->setText(QString::number(info.K[7]));
    ui->labelMatDis22->setText(QString::number(info.K[8]));
    
    ui->labelMatRect00->setText(QString::number(info.P[0]));
    ui->labelMatRect01->setText(QString::number(info.P[1]));
    ui->labelMatRect02->setText(QString::number(info.P[2]));
    ui->labelMatRect03->setText(QString::number(info.P[3]));
    
    ui->labelMatRect10->setText(QString::number(info.P[4]));
    ui->labelMatRect11->setText(QString::number(info.P[5]));
    ui->labelMatRect12->setText(QString::number(info.P[6]));
    ui->labelMatRect13->setText(QString::number(info.P[7]));
    
    ui->labelMatRect20->setText(QString::number(info.P[8]));
    ui->labelMatRect21->setText(QString::number(info.P[9]));
    ui->labelMatRect22->setText(QString::number(info.P[10]));
    ui->labelMatRect23->setText(QString::number(info.P[11]));
    

}

void task(sensor_msgs::CameraInfo camInfo, std::string topicName)
{
    int zero = 0;
    ros::init(zero, 0, "infopub");
    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<sensor_msgs::CameraInfo>(topicName, 20);
    
    while(true)
    {
        using namespace std::chrono_literals;

        std::this_thread::sleep_for(100ms);
        
        publisher.publish(camInfo);
    }
}

void MainWindow::slotPushButtonPublishCamInfo()
{
    //Start to continously publish the cam infos
    QString topicNameqs = ui->lineEditTopicName->text();
    std::string topicName = topicNameqs.toStdString();
    std::thread t(task, info, topicName);
    t.detach();
    
    ui->pushButtonPublishCamInfo->setEnabled(false);
    ui->lineEditTopicName->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

