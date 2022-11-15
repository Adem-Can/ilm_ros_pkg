#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "GUIUpdater.h"

#include <ilm_ros_pkg/Color.h>
#include <ilm_ros_pkg/ColorArray.h>
#include <ilm_ros_pkg/ColoredPointcloud.h>

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>

#include <QFileDialog>
#include <QMessageBox>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>

#define UNBOXIMAGE true

static ros::Publisher* publisherImagePtr = NULL;
static ros::Publisher* publisherImagePtr2 = NULL;
static ros::Publisher* publishercpcPtr = NULL;

sensor_msgs::Image imageBuf;
sensor_msgs::Image imageBuf2;

bool vizPoints;

std::mutex lock;
std::mutex lock2;

cv::Mat rvecParam;
cv::Mat tvecParam;
cv::Matx33d intrinsicsParam;

cv::Mat rvecParam2;
cv::Mat tvecParam2;
cv::Matx33d intrinsicsParam2;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    GUIUpdater::init();
    
    //Connect GUI updater to MainWIndow slots
    QObject::connect(
    GUIUpdater::_updaterInstance,
    SIGNAL(SIGNAL_updateLabelCount(std::string)), 
    this, 
    SLOT(modSlotUpdateLabelCount(std::string))
    );
    
    QObject::connect(
    GUIUpdater::_updaterInstance,
    SIGNAL(SIGNAL_updateLabelCountI(std::string)), 
    this, 
    SLOT(modSlotUpdateLabelCountI(std::string))
    );
    
    QObject::connect(
    GUIUpdater::_updaterInstance,
    SIGNAL(SIGNAL_updateLabelCountI2(std::string)), 
    this, 
    SLOT(modSlotUpdateLabelCountI2(std::string))
    );
    
    // QObject::connect(
    // GUIUpdater::_updaterInstance,
    // SIGNAL(SIGNAL_setUiEnabled(bool)), 
    // this, 
    // SLOT(modSlotSetUiEnabled(bool))
    // );
    
    _stop = false;
    
    //Init ROS
    int zero = 0;
    ros::init(zero, 0, "echo_test");
    _nodePtr = new ros::NodeHandle();
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
    sensor_msgs::CameraInfo info;
    
    bool success = camera_calibration_parsers::readCalibration(stddir, camname, info);
    
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
    //set ui
    
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

    ui->pushButtonStart->setEnabled(true);
}

//Parse a camera info file that the solvePNP function is going to use
void MainWindow::slotPushButtonOpenFile2()
{
    ui->pushButtonOpenFile_2->setEnabled(false);
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
    sensor_msgs::CameraInfo info;
    
    bool success = camera_calibration_parsers::readCalibration(stddir, camname, info);
    
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
    ui->checkBoxRect_2->setEnabled(false);

    if(ui->checkBoxRect_2->checkState() == Qt::Unchecked)
    {
        std::cout << "Using raw image with distortion parameters" << std::endl;
        cv::Matx33d mat(
        info.K[0], info.K[1], info.K[2],
        info.K[3], info.K[4], info.K[5],
        info.K[6], info.K[7], info.K[8]
        );

        intrinsics2 = mat;
    }
    else
    {
        std::cout << "Using rectified image without distortion parameters" << std::endl;

        cv::Matx33d mat(
        info.P[0], info.P[1], info.P[2],
        info.P[4], info.P[5], info.P[6],
        info.P[8], info.P[9], info.P[10]
        );

        intrinsics2 = mat;
    }

    distortion2 = info.D;
    //set ui
    
    ui->checkBoxRect_2->setEnabled(false);

    if(ui->checkBoxRect_2->checkState() == Qt::Unchecked)
    {
        std::cout << "Using raw image with distortion parameters" << std::endl;
        cv::Matx33d mat(
        info.K[0], info.K[1], info.K[2],
        info.K[3], info.K[4], info.K[5],
        info.K[6], info.K[7], info.K[8]
        );

        intrinsics2 = mat;
    }
    else
    {
        std::cout << "Using rectified image without distortion parameters" << std::endl;

        cv::Matx33d mat(
        info.P[0], info.P[1], info.P[2],
        info.P[4], info.P[5], info.P[6],
        info.P[8], info.P[9], info.P[10]
        );

        intrinsics2 = mat;
    }

    distortion2 = info.D;

    ui->pushButtonStart->setEnabled(true);
}

// ####################################################
// ########## Callback functions for ROS ##############
// ####################################################

void ROSCallbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    static int counterI = 0;
    counterI++;
    std::string counterI_s = std::to_string(counterI);
    GUIUpdater::_updaterInstance->updateLabelCountI(counterI_s);

    //Copy message to local stack
    sensor_msgs::Image message = *(msg.get());

    //Copy message to the output container
    lock.lock();
    imageBuf = message;
    lock.unlock();
}

void ROSCallbackImage2(const sensor_msgs::Image::ConstPtr& msg)
{
    static int counterI2 = 0;
    counterI2++;
    std::string counterI_s = std::to_string(counterI2);
    GUIUpdater::_updaterInstance->updateLabelCountI2(counterI_s);

    //Copy message to local stack
    sensor_msgs::Image message = *(msg.get());

    //Copy message to the output container
    lock2.lock();
    imageBuf2 = message;
    lock2.unlock();
}

void ROSCallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    static int counterPC = 0;
    counterPC++;
    std::string counterPC_s = std::to_string(counterPC);
    GUIUpdater::_updaterInstance->updateLabelCount(counterPC_s);

    sensor_msgs::Image imageMsg;
    sensor_msgs::Image imageMsg2;

    lock.lock();
    imageMsg = imageBuf;
    lock.unlock();
    lock2.lock();
    imageMsg2 = imageBuf2;
    lock2.unlock();

    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr2;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    try
    {
        cv_ptr2 = cv_bridge::toCvCopy(imageMsg2, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    //Setup complete. Mark the points in the image.

    //Extract the points from the point 
    //cloud and project them into the image

    //All of the above values are expected 
    //to be constant since we work with the same
    //Blickfeld Lidar all the time, so these
    // values will be hardcoded below.
    //(except width, which varies for some reason)
    
    //Unbox the data
    //Get a float pointer that points to the
    //start of the data array
    //(float pointer makes more sense than uin8_t pointer,
    //since we know that the data is comprised of
    //32 bit floats. The occasional 32 bit integer 
    //for intensity can be easily converted)
    float* data_ptr = (float*)(&((msg->data).at(0)));
    
    //width varies in every message; width refers to
    //the number of points; each point consists of
    //4 32-bit values. (3 floats and an int)
    int numberOfPoints = msg->width;
    //numberOfValues refers to how many 32-bit values there are
    //in total. for example, if there are 1000 points 
    //in the cloud, there are 4000 32-bit values
    int numberOfValues = numberOfPoints * 4;
    
    //Container for the 3d points
    std::vector<cv::Point3d> p3d;
    //Every point has 4 32-bit values. Iterate over them at once.
    for(int i = 0; i < numberOfValues; i += 4)
    {
        float x = data_ptr[i + 0];
        float y = data_ptr[i + 1];
        float z = data_ptr[i + 2];
        uint32_t intensity = *((uint32_t*) (data_ptr + i + 3));

        //Convert from ROS coordinate frame to OpenCV coordinate frame
        p3d.push_back(cv::Point3d(x, -z, y));
    }

    //Colored pointcloud message (final result of fusion)
    ilm_ros_pkg::ColoredPointcloud cpc;
    //Copy pointcloud to local stack
    sensor_msgs::PointCloud2 pclocal = *(msg.get());
    cpc.pointCloud = pclocal;

    std::vector<cv::Point2d> p2d;
    cv::projectPoints(p3d, rvecParam, tvecParam, intrinsicsParam, cv::noArray(), p2d);

    if(p2d.size() == p3d.size())
    {
        //std::cout << "size is good" << std::endl;
    }
    else
    {
        std::cout << "SIZE MISMATCH" << std::endl;
    }
    
    //the color at image.at(p2d[i]) corresponds to the point at p3d[i]
    int height1 = cv_ptr->image.size().height;
    int width1 = cv_ptr->image.size().width;

    ilm_ros_pkg::ColorArray colorArray1;
    for(int j = 0; j < p2d.size(); j++)
    {
        //std::cout << "image 1 point at x " << p2d.at(j).x << " y " << p2d.at(j).y << std::endl;

        int x = (int)p2d.at(j).x;
        int y = (int)p2d.at(j).y;

        ilm_ros_pkg::Color colorBuf;

        if((x < 0) || (x >= width1) || (y < 0) || (y >= height1))
        {
            colorBuf.red = 0;
            colorBuf.green = 0;
            colorBuf.blue = 0;
            colorArray1.colors.push_back(colorBuf);
            colorArray1.valid.push_back(false);
            //std::cout << "no color" << std::endl;
            //The 3D point from the point cloud can not be seen in this image
            continue;
        }

        cv::Vec3b color;
        color = cv_ptr->image.at<cv::Vec3b>(y,x);

        //OpenCV is BGR not RGB
        colorBuf.red = color[2];
        colorBuf.green = color[1];
        colorBuf.blue = color[0];
        colorArray1.colors.push_back(colorBuf);
        colorArray1.valid.push_back(true);

        //std::cout << "color: " << ((int)(color[0])) << ", " << ((int)(color[1])) << ", " << ((int)(color[2])) << std::endl;

        //Do this at the END
        if(vizPoints)
        {
            cv::circle(cv_ptr->image, p2d.at(j), 1, cv::Scalar(0.0, 255.0, 0.0), 1);
        }
    }
    cpc.colorscam1 = colorArray1;
    
    std::vector<cv::Point2d> p2d2;
    cv::projectPoints(p3d, rvecParam2, tvecParam2, intrinsicsParam2, cv::noArray(), p2d2);
    
    //the color at image.at(p2d[i]) corresponds to the point at p3d[i]
    int height2 = cv_ptr2->image.size().height;
    int width2 = cv_ptr2->image.size().width;

    ilm_ros_pkg::ColorArray colorArray2;
    for(int j = 0; j < p2d2.size(); j++)
    {
        //std::cout << "image 2 point at x " << p2d2.at(j).x << " y " << p2d2.at(j).y << std::endl;

        int x = (int)p2d2.at(j).x;
        int y = (int)p2d2.at(j).y;

        ilm_ros_pkg::Color colorBuf;

        if((x < 0) || (x >= width2) || (y < 0) || (y >= height2))
        {
            colorBuf.red = 0;
            colorBuf.green = 0;
            colorBuf.blue = 0;
            colorArray2.colors.push_back(colorBuf);
            colorArray2.valid.push_back(false);
            //std::cout << "no color" << std::endl;
            //The 3D point from the point cloud can not be seen in this image
            continue;
        }

        cv::Vec3b color;
        color = cv_ptr2->image.at<cv::Vec3b>(y,x);

        //OpenCV is BGR not RGB
        colorBuf.red = color[2];
        colorBuf.green = color[1];
        colorBuf.blue = color[0];
        colorArray2.colors.push_back(colorBuf);
        colorArray2.valid.push_back(true);

        //std::cout << "color: " << ((int)(color[0])) << ", " << ((int)(color[1])) << ", " << ((int)(color[2])) << std::endl;

        //Do this at the END
        if(vizPoints)
        {
            cv::circle(cv_ptr2->image, p2d2.at(j), 1, cv::Scalar(0.0, 255.0, 0.0), 1);
        }
        
    }

    cpc.colorscam2 = colorArray2;
    
    // cv::imshow("window", cv_ptr->image);

    publisherImagePtr->publish(cv_ptr->toImageMsg());
    publisherImagePtr2->publish(cv_ptr2->toImageMsg());
    publishercpcPtr->publish(cpc);
}

// ################################
// ########## SLOTS ###############
// ################################

// ####################################################
// ########## Task for thread #########################
// ####################################################

void task(std::atomic<bool>* stop_ptr, ros::NodeHandle* nodePtr, std::string imgtopic, std::string imgtopic2, std::string pctopic)
{
    using namespace std::chrono_literals;
    GUIUpdater::_updaterInstance->setUiEnabled(false);
    
    //subscribe to topics
    ros::Subscriber subscriberPointCloud    = nodePtr->subscribe(pctopic, 10, ROSCallbackPointCloud);
    ros::Subscriber subscriberImage         = nodePtr->subscribe(imgtopic, 10, ROSCallbackImage);
    ros::Subscriber subscriberImage2         = nodePtr->subscribe(imgtopic2, 10, ROSCallbackImage2);
    
    //define own topics to echo to
    ros::Publisher publisherImage       = nodePtr->advertise<sensor_msgs::Image>("protofuse/image_preview", 1000);
    
    //define own topics to echo to
    ros::Publisher publisherImage2       = nodePtr->advertise<sensor_msgs::Image>("protofuse/image_preview2", 1000);

    ros::Publisher publishercpc         = nodePtr->advertise<ilm_ros_pkg::ColoredPointcloud>("protofuse/result", 1000);
    
    //setup the publishers so the callback functions can use them
    publisherImagePtr = &publisherImage;
    publisherImagePtr2 = &publisherImage2;
    publishercpcPtr = &publishercpc;
    
    //start echoing
    std::cout << "Listening..." << std::endl;

    //Never stop.
    while(true)
    {
        ros::spinOnce();
        std::this_thread::sleep_for(10ms);
    }
}

void MainWindow::slotButtonStartPressed()
{
    double rxval = ui->rx->value();
    double ryval = ui->ry->value();
    double rzval = ui->rz->value();
    double txval = ui->tx->value();
    double tyval = ui->ty->value();
    double tzval = ui->tz->value();
    
    double rxval2 = ui->rx_2->value();
    double ryval2 = ui->ry_2->value();
    double rzval2 = ui->rz_2->value();
    double txval2 = ui->tx_2->value();
    double tyval2 = ui->ty_2->value();
    double tzval2 = ui->tz_2->value();
    
    bool viz = (ui->checkBoxViz->checkState() == Qt::Checked);
    vizPoints = viz;

    cv::Mat rvec_est = (cv::Mat_<double>(3,1) << rxval, ryval, rzval);
    cv::Mat tvec_est = (cv::Mat_<double>(3,1) << txval, tyval, tzval);
    
    cv::Mat rvec_est2 = (cv::Mat_<double>(3,1) << rxval2, ryval2, rzval2);
    cv::Mat tvec_est2 = (cv::Mat_<double>(3,1) << txval2, tyval2, tzval2);

    _rvec = rvec_est;
    _tvec = tvec_est;
    
    _rvec2 = rvec_est2;
    _tvec2 = tvec_est2;

    QString imgtopicqs = ui->lineEditTopicImage->text();
    std::string imgtopic = imgtopicqs.toStdString();
    
    QString imgtopicqs2 = ui->lineEditTopicImage_2->text();
    std::string imgtopic2 = imgtopicqs2.toStdString();

    QString pctopicqs = ui->lineEditTopicPointcloud->text();
    std::string pctopic = pctopicqs.toStdString();

    rvecParam = _rvec;
    tvecParam = _tvec;
    intrinsicsParam = intrinsics;
    
    rvecParam2 = _rvec2;
    tvecParam2 = _tvec2;
    intrinsicsParam2 = intrinsics2;

    std::thread t(task, &_stop, _nodePtr, imgtopic, imgtopic2, pctopic);
    t.detach();

    ui->rx->setEnabled(false);
    ui->ry->setEnabled(false);
    ui->rz->setEnabled(false);
    ui->tx->setEnabled(false);
    ui->ty->setEnabled(false);
    ui->tz->setEnabled(false);
    ui->rx_2->setEnabled(false);
    ui->ry_2->setEnabled(false);
    ui->rz_2->setEnabled(false);
    ui->tx_2->setEnabled(false);
    ui->ty_2->setEnabled(false);
    ui->tz_2->setEnabled(false);
    ui->pushButtonStart->setEnabled(false);
}

// ################################
// ########## MOD SLOTS ###########
// ################################

void MainWindow::modSlotUpdateLabelCount(std::string count)
{
    ui->labelCount->setText(QString::fromStdString(count));
}

void MainWindow::modSlotUpdateLabelCountI(std::string count)
{
    ui->labelCountI->setText(QString::fromStdString(count));
}

void MainWindow::modSlotUpdateLabelCountI2(std::string count)
{
    ui->labelCount_2->setText(QString::fromStdString(count));
}
// void MainWindow::modSlotSetUiEnabled(bool enabled)
// {
//     if(enabled)
//     {
//         ui->pushButtonStart->setEnabled(true);
//         // ui->pushButtonStop->setEnabled(false);
//     }
//     else
//     {
//         ui->pushButtonStart->setEnabled(false);
//         // ui->pushButtonStop->setEnabled(true);
//     }
    
// }

MainWindow::~MainWindow()
{
    delete ui;
}

