#pragma once

#ifndef __H_INCLUDE_GENERAL__
#define __H_INCLUDE_GENERAL__

#define _USE_MATH_DEFINES
#include <math.h> // definition of M_PI

#include <Eigen/Core>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "exceptions.h"

using namespace std;
using namespace std::chrono;

//OpenGL stuff
#define __CELL_WIDTH_GLUT                      2.0         // in meters
#define __STEP_POINT_CLOUD_DECIM                 3

#endif
