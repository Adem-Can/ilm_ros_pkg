#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
    void slotPushButtonPublishCamInfo();

private:
    Ui::MainWindow *ui;
    sensor_msgs::CameraInfo info;
};
#endif // MAINWINDOW_H
