#pragma once

#include "GUIUpdater.h"
#include "SimplePoint3D.h"

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <QMainWindow>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <atomic>
#include <vector>
#include <utility>

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
    void slotButtonStartPressed();
    void slotButtonStopPressed();
    void slotButtonAddPairPressed();
    void slotButtonExtractDataPressed();
    void slotaddpair();

    void slotimgleft();
    void slotimgup();
    void slotimgdown();
    void slotimgright();
    void slotsetimgx(int x);
    void slotsetimgy(int y);

    void slotpcnext();
    void slotpcprev();
    void slotsetpc(int i);

    void slotwrite();
    
    void modSlotUpdateLabelCount(std::string count);
    void modSlotUpdateLabelCountI(std::string count);
    void modSlotSetUiState(int state);

private:
    Ui::MainWindow *ui;
    ros::NodeHandle* _nodePtr;
    
    std::atomic<bool> _stop;

    ros::Publisher _publisherPointCloud;
    ros::Publisher _publisherImage;
    ros::Publisher _publisherPoint;

    sensor_msgs::Image _image;
    sensor_msgs::PointCloud2 _pointCloud;

    std::vector<SimplePoint3D> _points;
    cv::Mat _imageMat;

    int pcSize;
    int pcSelected;

    int imgWidth;
    int imgHeight;
    int imgSelectedHeight;
    int imgSelectedWidth;

    void selectImage(int x, int y);
    void selectPointcloud(int i);

    std::vector<std::pair<SimplePoint3D, cv::Point2i>> _mappings;
};
