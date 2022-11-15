#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "GUIUpdater.h"
#include "SimplePoint3D.h"

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <fstream>

#define OUTPUT_PATH_POINTCLOUD "/home/ilmu011/Desktop/pointcloud.txt"
#define OUTPUT_PATH_IMAGE "/home/ilmu011/Desktop/image.png"

//The ROS callback function can't be given any additional arguments
//so it will use these global variables to publish messages
static ros::Publisher* publisherPointCloudPtr = NULL;
static ros::Publisher* publisherImagePtr = NULL;

//The ROS callback function can't be given any additional arguments
//so it will use these global variables to store its output
static sensor_msgs::Image* imagePtr = NULL;
static sensor_msgs::PointCloud2* pointCloudPtr = NULL;

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
    SIGNAL(SIGNAL_setUiState(int)), 
    this, 
    SLOT(modSlotSetUiState(int))
    );
    
    //Init the shared atomic bool
    _stop = false;
    
    //Init ROS
    int zero = 0;
    ros::init(zero, 0, "echo_test");

    _nodePtr = new ros::NodeHandle();

    //setup the global variables for the ROS callback functions
    imagePtr = &_image;
    pointCloudPtr = &_pointCloud;

    _publisherPointCloud  = _nodePtr->advertise<sensor_msgs::PointCloud2>("extrinsicCalib/pointCloudPreview", 1000);
    _publisherImage       = _nodePtr->advertise<sensor_msgs::Image>("extrinsicCalib/imagePreview", 1000);
    _publisherPoint       = _nodePtr->advertise<geometry_msgs::Point>("extrinsicCalib/pointSelectPreview", 1000);

    publisherPointCloudPtr = &_publisherPointCloud;
    publisherImagePtr = &_publisherImage;

    pcSize = 0;
    pcSelected = 0;

    imgWidth = 0;
    imgHeight = 0;
    imgSelectedHeight = 0;
    imgSelectedWidth = 0;
}

// ####################################################
// ########## Callback functions for ROS ##############
// ####################################################

void ROSCallbackImage(const sensor_msgs::Image::ConstPtr& message)
{
    //Received a message: update the GUI
    static int counterI = 0;
    counterI++;
    std::string counterI_s = std::to_string(counterI);
    GUIUpdater::_updaterInstance->updateLabelCountI(counterI_s);

    //Copy message to local stack
    sensor_msgs::Image msg = *(message.get());

    //Copy message to the output container
    imagePtr[0] = msg;
    
    //Echo the message
    publisherImagePtr->publish(msg);
}

void ROSCallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& message)
{
    //Received a message: update the GUI
    static int counterPC = 0;
    counterPC++;
    std::string counterPC_s = std::to_string(counterPC);
    GUIUpdater::_updaterInstance->updateLabelCount(counterPC_s);
    
    //Copy message to local stack
    sensor_msgs::PointCloud2 msg = *(message.get());

    //Copy message to the output container
    pointCloudPtr[0] = msg;

    //Echo the message
    publisherPointCloudPtr->publish(msg);
}

// ####################################################
// ########## Task for thread #########################
// ####################################################

void task(std::atomic<bool>* stop_ptr, ros::NodeHandle* nodePtr, std::string imageTopic, std::string pointCloudTopic)
{
    using namespace std::chrono_literals;
    GUIUpdater::_updaterInstance->setUiState(0);
    
    //subscribe to topics
    ros::Subscriber subscriberPointCloud    = nodePtr->subscribe(pointCloudTopic, 10, ROSCallbackPointCloud);
    ros::Subscriber subscriberImage         = nodePtr->subscribe(imageTopic, 10, ROSCallbackImage);
    
    //start echoing
    std::cout << "Listening..." << std::endl;
    while((*stop_ptr) == false)
    {
        ros::spinOnce();
        std::this_thread::sleep_for(10ms);
    }
    //stop echoing
    std::cout << "Done listening" << std::endl;
    std::this_thread::sleep_for(1s);
    *stop_ptr = false;

    GUIUpdater::_updaterInstance->setUiState(1);
}

// ################################
// ########## SLOTS ###############
// ################################

//These slot functions are called when elements
//in this window are interacted with by the user

void MainWindow::slotButtonStartPressed()
{
    std::string imageTopic = ui->lineEditImageTopic->text().toStdString();
    std::string pointCloudTopic = ui->lineEditPointCloudTopic->text().toStdString();

    std::thread t(task, &_stop, _nodePtr, imageTopic, pointCloudTopic);
    t.detach();
}

void MainWindow::slotButtonStopPressed()
{
    _stop = true;

    ui->pushButtonStop->setEnabled(false);
}

void MainWindow::slotButtonExtractDataPressed()
{
    //Unbox pointcloud
    {
        sensor_msgs::PointCloud2 msg = _pointCloud;
        //Unbox message
        using std::cout;
        using std::endl;
        // cout << "=========== New point cloud message" << endl;
        //Unbox header
        std_msgs::Header header = msg.header;
        uint32_t seq = header.seq;
        ros::Time stamp = header.stamp;
        int32_t sec = stamp.sec;
        int32_t nsec = stamp.nsec;
        std::string frame_id = header.frame_id;
        
        // cout << "seq: " << seq << endl;
        // cout << "sec: " << sec << endl;
        // cout << "nsec: " << nsec << endl;
        // cout << "frame_id: " << frame_id << endl;
        //Unbox pointfield
        //A std::vector of sensor_msg::Pointfields
        //tells us how the data is formatted
        //Get the vector
        std::vector<sensor_msgs::PointField> fields = msg.fields;
        
        //Look how many fields there are per entry (an entry 
        //refers to a single point in the point cloud)
        //Note all of the below values are expected to be 
        //constant (except width, which varies for some reason)
        int fieldcount = fields.size();
        // cout << "fields: " << fieldcount << endl;
        
        //Analyze the fields
        for(int i = 0; i < fieldcount; i++)
        {
            sensor_msgs::PointField field = fields.at(i);
            
            std::string name = field.name;
            uint32_t offset = field.offset;
            uint8_t datatype = field.datatype;
            uint32_t count = field.count;
            
            // cout << "name: " << name << endl;
            // cout << "offset: " << offset << endl;
            // cout << "datatype: " << (uint32_t)datatype << endl;
            // cout << "count: " << count << endl;
            // cout << "-------" << endl;
        }
        
        //Unbox rest of the message
        uint32_t height = msg.height;
        uint32_t width = msg.width;
        bool bigendian = msg.is_bigendian;
        uint32_t point_step = msg.point_step;
        uint32_t row_step = msg.row_step;
        bool dense = msg.is_dense;
        
        // cout << "height: " << height << endl;
        // cout << "width: " << width << endl;
        // cout << "bigendian: " << std::boolalpha << bigendian << endl;
        // cout << "point_step: " << point_step << endl;
        // cout << "row_step: " << row_step << endl;
        // cout << "dense: " << std::boolalpha << dense << endl;
        
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
        float* data_ptr = (float*)(&((msg.data).at(0)));
        
        //width varies in every message; width refers to
        //the number of points; each point consists of
        //4 32-bit values. (3 floats and an int)
        int numberOfPoints = width;
        //numberOfValues refers to how many 32-bit values there are
        //in total. for example, if there are 1000 points 
        //in the cloud, there are 4000 32-bit values
        int numberOfValues = numberOfPoints * 4;
        
        // std::ofstream output(OUTPUT_PATH_POINTCLOUD, std::ofstream::trunc);
        
        //Every point has 4 32-bit values. Iterate over them at once.
        for(int i = 0; i < numberOfValues; i += 4)
        {
            float x = data_ptr[i + 0];
            float y = data_ptr[i + 1];
            float z = data_ptr[i + 2];
            uint32_t intensity = *((uint32_t*) (data_ptr + i + 3));

            SimplePoint3D point;
            point.x = x;
            point.y = y;
            point.z = z;

            _points.push_back(point);
            // output << "X: " << std::to_string(x) << endl;
            // output << "Y: " << std::to_string(y) << endl;
            // output << "Z: " << std::to_string(z) << endl;
            // output << "I: " << std::to_string(intensity) << endl;
            // output << "-------------" << endl;
        }
        // output.close();
    }

    //Unbox image
    {
        sensor_msgs::Image msg = _image;
        //Unbox message
        using std::cout;
        using std::endl;
        // cout << "=========== New image message" << endl;
        //Unbox header
        std_msgs::Header header = msg.header;
        uint32_t seq = header.seq;
        ros::Time stamp = header.stamp;
        int32_t sec = stamp.sec;
        int32_t nsec = stamp.nsec;
        std::string frame_id = header.frame_id;
        
        // cout << "seq: " << seq << endl;
        // cout << "sec: " << sec << endl;
        // cout << "nsec: " << nsec << endl;
        // cout << "frame_id: " << frame_id << endl;
        
        //Unbox rest of the message
        uint32_t height = msg.height;
        uint32_t width = msg.width;
        uint8_t bigendian = msg.is_bigendian; //(why isn't this bool??)
        uint32_t step = msg.step;
        std::string encoding = msg.encoding;
        
        // cout << "encoding: " << encoding << endl;
        // cout << "height: " << height << endl;
        // cout << "width: " << width << endl;
        // cout << "bigendian: " << (int) bigendian << endl;
        // cout << "step: " << step << endl;
        //The encoding is expected to be constant, so it will be hardcoded.
        
        //Unbox the data
        //Make a pointer to the data (void* because OpenCV expects a void pointer)
        void* data_ptr = (void*)(&((msg.data).at(0)));
        
        //Setup the OpenCV container
        cv::Mat image(cv::Size(width, height), CV_8UC3, data_ptr);
        
        //The data actually is already in RGB form, but OpenCV automatically assumes BGR
        //So, flipping it will make OpenCV display it correctly.
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        _imageMat = image.clone();
        
        //Write it to file
        //cv::imwrite(OUTPUT_PATH_IMAGE, image);
        
        //Display in real time
        // cv::imshow("Image", image);
    }

    //Now the data is available in much simpler form.
    pcSize = _points.size();
    imgWidth = _imageMat.size().width;
    imgHeight = _imageMat.size().height;

    std::cout << "Data extracted." << std::endl;
    std::cout << "Number of points: " << pcSize << std::endl;
    std::cout << "Image: " << imgWidth << "x" << imgHeight << std::endl;

    ui->labelimgmaxx->setText(QString::number(imgWidth));
    ui->labelimgmaxy->setText(QString::number(imgHeight));
    ui->labelpcmax->setText(QString::number(pcSize));
    modSlotSetUiState(2);
}

void MainWindow::slotButtonAddPairPressed()
{
    _publisherImage.publish(_image);
    _publisherPointCloud.publish(_pointCloud);
}

// ################################
// ########## MOD SLOTS ###########
// ################################

//These slot functions are called through the
//GUI updater when another thread wants to
//modify UI elements of this window

void MainWindow::modSlotUpdateLabelCount(std::string count)
{
    ui->labelCount->setText(QString::fromStdString(count));
}

void MainWindow::modSlotUpdateLabelCountI(std::string count)
{
    ui->labelCountI->setText(QString::fromStdString(count));
}

//Enable/disable UI elements based on program state
//This also acts as a helper slot function for the external thread
void MainWindow::modSlotSetUiState(int state)
{
    switch(state)
    {
        case 0:
        {
            ui->pushButtonStart->setEnabled(false);
            ui->lineEditImageTopic->setEnabled(false);
            ui->lineEditPointCloudTopic->setEnabled(false);
            ui->pushButtonStop->setEnabled(true);
            break;
        }
        case 1:
        {
            ui->pushButtonStop->setEnabled(false);
            ui->pushButtonExractData->setEnabled(true);
            break;
        }
        case 2:
        {
            ui->pushButtonExractData->setEnabled(false);
            ui->pushButtonwritetofile->setEnabled(true);
            ui->pushButtonAddPair->setEnabled(true);
            ui->pushButtonimgup->setEnabled(true);
            ui->pushButtonimgdown->setEnabled(true);
            ui->pushButtonimgleft->setEnabled(true);
            ui->pushButtonimgright->setEnabled(true);
            ui->spinBoximgx->setEnabled(true);
            ui->spinBoximgy->setEnabled(true);
            ui->pushButtonpcprev->setEnabled(true);
            ui->pushButtonpcnext->setEnabled(true);
            ui->spinBoxpcindex->setEnabled(true);
        }
    }
}

    void MainWindow::slotimgleft()
    {
        ui->spinBoximgx->setValue(imgSelectedWidth - 1);
    }

    void MainWindow::slotimgup()
    {
        ui->spinBoximgy->setValue(imgSelectedHeight - 1);
    }

    void MainWindow::slotimgdown()
    {
        ui->spinBoximgy->setValue(imgSelectedHeight + 1);
    }

    void MainWindow::slotimgright()
    {
        ui->spinBoximgx->setValue(imgSelectedWidth + 1);
    }

    void MainWindow::slotsetimgx(int x)
    {
        selectImage(x, imgSelectedHeight);
    }

    void MainWindow::slotsetimgy(int y)
    {
        selectImage(imgSelectedWidth, y);
    }

    void MainWindow::slotpcnext()
    {
        int newValue = pcSelected + 1;

        if(newValue >= pcSize)
        {
            newValue = pcSize - 1;
        }

        ui->spinBoxpcindex->setValue(newValue);
    }

    void MainWindow::slotpcprev()
    {
        int newValue = pcSelected - 1;

        if(newValue < 0)
        {
            newValue = 0;
        }

        ui->spinBoxpcindex->setValue(newValue);
    }

    void MainWindow::slotsetpc(int i)
    {
        selectPointcloud(i);
    }

    void MainWindow::slotaddpair()
    {
        SimplePoint3D point3;
        point3.x = 0.0f;
        point3.y = -1.0f;
        point3.z = 1.0f;
        cv::Point2i point2(imgSelectedWidth, imgSelectedHeight);
        _mappings.push_back(std::pair(point3, point2));

        _publisherPointCloud.publish(_pointCloud);
    }

    void MainWindow::slotwrite()
    {
        std::ofstream output("/home/ilmu011/Desktop/mappings.txt", std::ofstream::trunc);
        for(auto p : _mappings)
        {
            SimplePoint3D point3 = p.first;
            cv::Point2i point2 = p.second;

            output << std::to_string(point3.x) << std::endl;
            output << std::to_string(point3.y) << std::endl;
            output << std::to_string(point3.z) << std::endl;
            output << std::to_string(point2.x) << std::endl;
            output << std::to_string(point2.y) << std::endl;
        }
        output.close();
    }

// #################################################################
// ##########HELPER FUNCTIONS FOR SELECTION PROCESS#################
// #################################################################

void MainWindow::selectImage(int x, int y)
{
    imgSelectedHeight = y;
    imgSelectedWidth = x;

    cv::Mat imgcopy = _imageMat.clone();
    cv::circle(imgcopy, cv::Point(x,y),10,cv::Scalar(0, 255, 0));

    cv::imshow("window", imgcopy);
}

void MainWindow::selectPointcloud(int i)
{
    pcSelected = i;

    SimplePoint3D selected = _points.at(pcSelected);

    //Publish selected point
    geometry_msgs::Point msg;

    msg.x = selected.x;
    msg.y = selected.y;
    msg.z = selected.z;

    ui->labelpvalx->setText(QString::number(msg.x));
    ui->labelpvaly->setText(QString::number(msg.z));
    ui->labelpvalz->setText(QString::number(msg.y));

    ui->labelpvalx_2->setText(QString::number(msg.x));
    ui->labelpvaly_2->setText(QString::number(msg.y));
    ui->labelpvalz_2->setText(QString::number(msg.z));

    _publisherPoint.publish(msg);
    _publisherPointCloud.publish(_pointCloud);
}

// ################################
// ################################
// ################################



MainWindow::~MainWindow()
{
    delete ui;
}

