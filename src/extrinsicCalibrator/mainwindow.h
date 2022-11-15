#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
public slots:
    void slotPushButtonOpenFile();
    void slotButtonSelectInputDir();
    void slotPushButtonROS2CV();
    void slotCalibrate();

private:
    Ui::MainWindow *ui;
    sensor_msgs::CameraInfo info;

    cv::Matx33d intrinsics;
    std::vector<double> distortion;

    std::vector<cv::Point3d> _points;
    std::vector<cv::Point2d> _pixels;

    bool haveIntrinsics;
    bool havePairs;
};
#endif // MAINWINDOW_H
