#pragma once

#ifndef __H_INCLUDE_GENERAL__
#define __H_INCLUDE_GENERAL__

#define _USE_MATH_DEFINES

#define PIGR 3.14159265
#define APPROACH_OFFSET 200.0
#define FINGER_OFFSET 30.0 // mm

#include <math.h> // definition of M_PI
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <stdio.h>
#include <chrono>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

// ROS
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "cv_bridge/cv_bridge.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

// h/hpp files
#include "exceptions.h"

//OpenGL stuff
#define __CELL_WIDTH_GLUT                      2.0         // in meters
#define __STEP_POINT_CLOUD_DECIM                 3

#endif
