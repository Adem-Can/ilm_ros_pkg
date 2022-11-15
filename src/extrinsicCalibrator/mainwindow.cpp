#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <chrono>
#include <thread>

#include <QFileDialog>
#include <QMessageBox>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    haveIntrinsics = false;
    havePairs = false;
}

//Parse a camera info file that the solvePNP function is going to use
void MainWindow::slotPushButtonOpenFile()
{
    ui->pushButtonOpenFile->setEnabled(false);
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
    // Set the camera infos as member variables that are easily accessible by OpenCV
    info = infoBuf;
    ui->checkBoxRect->setEnabled(false);

    if(ui->checkBoxRect->checkState() == Qt::Unchecked)
    {
        std::cout << "Using raw image with distortion parameters" << std::endl;
        cv::Matx33d mat(
        info.K[0], info.K[1], info.K[2],
        info.K[3], info.K[4], info.K[5],
        info.K[6], info.K[7], info.K[8]
        );

        intrinsics = mat;
    }
    else
    {
        std::cout << "Using rectified image without distortion parameters" << std::endl;

        cv::Matx33d mat(
        info.P[0], info.P[1], info.P[2],
        info.P[4], info.P[5], info.P[6],
        info.P[8], info.P[9], info.P[10]
        );

        intrinsics = mat;
    }

    distortion = info.D;
    
    //Visualize cam info data
    
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
    
    haveIntrinsics = true;

    if(haveIntrinsics && havePairs)
    {
        ui->pushButtonCalibrate->setEnabled(true);
    }

}

void MainWindow::slotButtonSelectInputDir()
{
    //Parse the 3D-2D correspondences
    
    ui->pushButtonSelectInputDir->setEnabled(false);
    QString dir = QFileDialog::getExistingDirectory
    (
        this,
        tr("Open Directory"),
        "",
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if(!dir.isEmpty())
    {
        ui->labelInputDir->setText(dir);
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("No custom directory was selected.");
        msgBox.exec();
        return;
    }

    std::string pointsPath = dir.toStdString() + "/points.txt";
    std::string pixelsPath = dir.toStdString() + "/pixels.txt";

    std::ifstream pointsFile(pointsPath);
    std::ifstream pixelsFile(pixelsPath);
    
    if(!pointsFile.is_open())
    {
        QMessageBox msgBox;
        msgBox.setText("points.txt could not be opened");
        msgBox.exec();
        pointsFile.close();
        return;
    }
    if(!pixelsFile.is_open())
    {
        QMessageBox msgBox;
        msgBox.setText("pixels.txt could not be opened");
        msgBox.exec();
        pixelsFile.close();
        return;
    }
    
    //Start reading
    std::vector<std::string> pointlines;
    std::vector<std::string> pixellines;
    std::string line;
    while(std::getline(pointsFile, line))
    {
        pointlines.push_back(line);
        //std::cout << line << std::endl;
    }
    while(std::getline(pixelsFile, line))
    {
        pixellines.push_back(line);
        //std::cout << line << std::endl;
    }
    
    //Strings are now stored
    //Check if size is correct
    
    if((pointlines.size() % 3) != 0)
    {
        QMessageBox msgBox;
        msgBox.setText("points.txt doesnt seem to be right size");
        msgBox.exec();
        pointsFile.close();
        return;
    }

    if((pixellines.size() % 2) != 0)
    {
        QMessageBox msgBox;
        msgBox.setText("pixels.txt doesnt seem to be right size");
        msgBox.exec();
        pixelsFile.close();
        return;
    }

    //How many points/pixels are there?
    int numpoints = pointlines.size() / 3;
    int numpixels = pixellines.size() / 2;

    // std::cout << numpoints << " points" << std::endl;
    // std::cout << numpixels << " pixels" << std::endl;
    
    
    //Extract all parameters
    std::vector<cv::Point3d> points;

    for(int i = 0; i < pointlines.size(); i += 3)
    {
        double x = static_cast<double>(std::stof(pointlines.at(i + 0)));
        double y = static_cast<double>(std::stof(pointlines.at(i + 1)));
        double z = static_cast<double>(std::stof(pointlines.at(i + 2)));

        points.push_back(cv::Point3d(x,y,z));
    }

        std::vector<cv::Point2d> pixels;

    for(int i = 0; i < pixellines.size(); i += 2)
    {
        double x = static_cast<double>(std::stof(pixellines.at(i + 0)));
        double y = static_cast<double>(std::stof(pixellines.at(i + 1)));

        pixels.push_back(cv::Point2d(x,y));
    }

    _pixels = pixels;
    _points = points;

    std::cout << "Parsed " << _pixels.size() << " pixels and " << _points.size() << " points" << std::endl;

    QString fpo = QString::fromStdString(std::to_string(_points.at(0).x) + " " + std::to_string(_points.at(0).y) + " " + std::to_string(_points.at(0).z));
    ui->labelfpo->setText(fpo);
    QString fpi = QString::fromStdString(std::to_string(_pixels.at(0).x) + " " + std::to_string(_pixels.at(0).y));
    ui->labelfpi->setText(fpi);

    pixelsFile.close();
    pointsFile.close();

    ui->pushButtonROS2CV->setEnabled(true);

    havePairs = true;

    if(haveIntrinsics && havePairs)
    {
        ui->pushButtonCalibrate->setEnabled(true);
    }
}

void MainWindow::slotPushButtonROS2CV()
{
    std::vector<cv::Point3d> points;
    //Convert ROS coordinates to OpenCV coordinates
    for(cv::Point3d point : _points)
    {
        points.push_back(cv::Point3d(point.x, -point.z, point.y));
    }
    _points = points;

    ui->pushButtonROS2CV->setEnabled(false);

    QString fpo = QString::fromStdString(std::to_string(_points.at(0).x) + " " + std::to_string(_points.at(0).y) + " " + std::to_string(_points.at(0).z));
    ui->labelfpo->setText(fpo);
}

void MainWindow::slotCalibrate()
{
    //Use the solvePNP function to calculate the extrinsic parameters with the Levenberg–Marquardt method
    cv::Matx33d intrParam = intrinsics;
    std::vector<cv::Point3d> p3d = _points;
    std::vector<cv::Point2d> p2d = _pixels;
    std::vector<double> dist = distortion;

    cv::Mat rvec_est = (cv::Mat_<double>(3,1) << 0.0, 0.0, 0.0);
    cv::Mat tvec_est = (cv::Mat_<double>(3,1) << 0.0, 0.0, 0.0);

    if(ui->checkBoxRect->checkState() == Qt::Checked)
    {
        cv::solvePnPRefineLM(p3d, p2d, intrParam, cv::noArray(), rvec_est, tvec_est);
    }
    else
    {
        cv::solvePnPRefineLM(p3d, p2d, intrParam, distortion, rvec_est, tvec_est);
    }

    std::cout << "rvec_est: " << rvec_est << std::endl;
    std::cout << "tvec_est: " << tvec_est << std::endl;
}

MainWindow::~MainWindow()
{
    delete ui;
}

