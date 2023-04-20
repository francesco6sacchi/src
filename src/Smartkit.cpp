#include "../include/Smartkit.hpp"

Smartkit::Smartkit(){
    ros::NodeHandle nh;

    // this->camera = new CameraManager(nh);
    this->object = new Object(nh);
    this->robot = new Robot(nh);
    this->planner = new GraspPlanner(nh);

    this->SmartkitLoop();

}

void Smartkit::SmartkitLoop(){

    int bestGraspIdx = -1;

    while(ros::ok()){

        Eigen::Matrix4d gripperPose;

        this->object->Tracking();

        // Eigen::Matrix4d tag2rgb; // va messo da qualche parte il tracking dell'aruco del robot

        // this->object->setPose2Robot(tag2rgb);

        if(bestGraspIdx >= 0){
            this->planner->NearGraspsRanking(this->object, bestGraspIdx);
        }
        else{
            this->planner->GraspsRanking(this->object);
        ROS_INFO("5");

        }

        if(!this->planner->getSortedGraspList()->empty()){

            // const moveit::core::JointModelGroup* approachJoints;
            Eigen::VectorXd approachJointValues;

            this->planner->BestGraspJoints(this->object, approachJointValues, bestGraspIdx);

        }
        else
            int bestGraspIdx = -1;


        sensor_msgs::JointState jointValuesRos;
        jointValuesRos.position.insert(jointValuesRos.position.end(), { approachJointValues(0), approachJointValues(1), approachJointValues(2), approachJointValues(3), approachJointValues(4), approachJointValues(5) } );
        
        this->robot->MoveRobot(approachJointValues, true); // joint target
        // this->robot->MoveRobot(this->planner->EigenToRosPose(gripperPose), true); // cartesian target

        this->robot->Spinner();
        this->object->Spinner();

    }
}