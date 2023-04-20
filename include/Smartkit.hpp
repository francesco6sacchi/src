#ifndef __H_INCLUDE_SMARTKIT__
#define __H_INCLUDE_SMARTKIT__


#include "../include/general.h"
#include "../include/geometry.h"
#include "../include/object.hpp"
#include "../include/camera.hpp"
#include "../include/robot.hpp"
#include "../include/grasp_planning.hpp"

class Smartkit{
public:
    Smartkit();
    void SmartkitLoop();

private:
    // CameraManager* camera;
    Object* object;
    Robot* robot;
    GraspPlanner* planner;


};

#endif