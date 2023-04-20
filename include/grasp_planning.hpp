#ifndef __H_INCLUDE_GRASP_PLANNING__
#define __H_INCLUDE_GRASP_PLANNING__

#include <string.h>

#include "general.h"
#include "object.hpp"
#include "robot.hpp"

class GraspPlanner{
private:

    // moveit variables
    // moveit::planning_interface::MoveGroupInterface* move_group;
    // robot_state::RobotState* robotState;
    const robot_state::JointModelGroup* joint_group;

    // ROS subscribers and variables
    ros::NodeHandle nh;
    ros::Subscriber subSafetyIndex;
    ros::Subscriber subBodyPointcloud;
    ros::Subscriber subJointState;
	sensor_msgs::PointCloud bodyPointcloud;

    // Methods
    Eigen::Matrix4d GetGripperPose(Object* obj, Grasp* grasp);
    double AxisSelection(Object* obj, Grasp grasp, Eigen::Matrix4d gripperPose);
    double ComputeF2(Object* pObj, Grasp grasp, Eigen::Vector3d vector, Eigen::Vector3d center);
    double ComputeF3(Eigen::VectorXd jointValues);
	double ComputeF4_OnApproach(Eigen::Matrix4d gripperPose);
	double ComputeF4(Eigen::VectorXd jointValues);
    double ComputeF5(Robot* robot);
	void VolumeCount(Object* obj, Grasp& grasp, Eigen::Vector3d& vector, Eigen::Vector3d& center);
    void RagagliaCoefficients(Robot* robot);

    // Data
    std::list<std::vector<double>> c_0, c_x, c_y;  // Ragaglia coefficients
    std::list<std::vector<Eigen::Vector3d>> ref_origins;  // mesh triangles reference origins
    std::vector<Grasp> sortedGraspList;
	Robot* pRobot;

public:
    // Constructor
    GraspPlanner(ros::NodeHandle nh);

    // Methods
    void GraspsRanking(Object* pObj);
    void NearGraspsRanking(Object* pObj, int& bestGraspIdx);
	std::vector<Grasp>* getSortedGraspList() { return &this->sortedGraspList; };
    void BestGraspJoints(Object* pObj, Eigen::VectorXd approachJointValues, int& bestGraspIdx);
    double cos_angle(Grasp& grasp, Eigen::Vector3d& point, Eigen::Vector3d& vector, Eigen::Vector3d& center);
    geometry_msgs::Pose EigenToRosPose(Eigen::Matrix4d poseMatrix);

    void Spinner();

};

#endif