#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

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
    void slotPushButtonOpenFile2();
    void slotButtonStartPressed();

    void modSlotUpdateLabelCount(std::string count);
    void modSlotUpdateLabelCountI(std::string count);
    void modSlotUpdateLabelCountI2(std::string count);
    // void modSlotSetUiEnabled(bool enabled);

private:
    Ui::MainWindow *ui;

    cv::Matx33d intrinsics;
    std::vector<double> distortion;
    
    cv::Matx33d intrinsics2;
    std::vector<double> distortion2;

    ros::NodeHandle* _nodePtr;
    
    std::atomic<bool> _stop;

    cv::Mat _rvec;
    cv::Mat _tvec;
    
    cv::Mat _rvec2;
    cv::Mat _tvec2;
};
#endif // MAINWINDOW_H
