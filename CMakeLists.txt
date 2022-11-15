cmake_minimum_required(VERSION 3.0.2)
project(ilm_ros_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH
"$ENV{QTDIR}"
"$ENV{QTDIR}/Qt6"
)   

find_package(OpenCV REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  camera_calibration_parsers
  message_generation
)

qt_standard_project_setup()

add_message_files(
    FILES
    Color.msg
    ColorArray.msg
    ColoredPointcloud.msg
)

#my own messges are dependent on these ROS messages
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ilm-ros-pkg
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs camera_calibration_parsers message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

####
####    Pair Picker
####

add_executable( pairPicker
                src/pairPicker/main.cpp
                src/pairPicker/mainwindow.cpp
                src/pairPicker/mainwindow.h
                src/pairPicker/mainwindow.ui
                src/pairPicker/GUIUpdater.h
                src/pairPicker/GUIUpdater.cpp
                src/pairPicker/SimplePoint3D.h
                )
target_link_libraries(pairPicker ${catkin_LIBRARIES} Qt6::Widgets)
target_link_libraries(pairPicker ${OpenCV_LIBS})

####
####    Extrinsic Calibrator
####

add_executable( extrinsicCalibrator
                src/extrinsicCalibrator/main.cpp
                src/extrinsicCalibrator/mainwindow.cpp
                src/extrinsicCalibrator/mainwindow.h
                src/extrinsicCalibrator/mainwindow.ui
                )
target_link_libraries(extrinsicCalibrator ${catkin_LIBRARIES} Qt6::Widgets)
target_link_libraries(extrinsicCalibrator ${OpenCV_LIBS})

####
####    Cam Info Publisher
####

add_executable( camInfoPublisher
                src/camInfoPublisher/main.cpp
                src/camInfoPublisher/mainwindow.cpp
                src/camInfoPublisher/mainwindow.h
                src/camInfoPublisher/mainwindow.ui
                )
target_link_libraries(camInfoPublisher ${catkin_LIBRARIES} Qt6::Widgets)
target_link_libraries(camInfoPublisher ${OpenCV_LIBS})

####
####    Proto Fuse
####

add_executable( protofuse
                src/protofuse/main.cpp
                src/protofuse/mainwindow.cpp
                src/protofuse/mainwindow.h
                src/protofuse/mainwindow.ui
                src/protofuse/GUIUpdater.h
                src/protofuse/GUIUpdater.cpp
                )
target_link_libraries(protofuse ${catkin_LIBRARIES} Qt6::Widgets)
target_link_libraries(protofuse ${OpenCV_LIBS})

