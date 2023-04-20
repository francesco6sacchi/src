#pragma once

#ifndef __H_INCLUDE_ROBOT__
#define __H_INCLUDE_ROBOT__

#include <list>
#include <math.h>

#include "general.h"
#include "geometry.h"

// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"


#include "dsr_msgs/Ikin.h"
#include "dsr_msgs/MoveJoint.h"
#include "ros/service_client.h"

#define __ROBOT_SPEED_STATUS_UNKNOWN                        0
#define __ROBOT_SPEED_STATUS_RUNNING                        1
#define __ROBOT_SPEED_STATUS_SAFETY_EXCEPTION               2
#define __ROBOT_SPEED_STATUS_RAMP_UP                        3

using namespace std;
using namespace std::chrono;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> JacobMatrix;

class Link {

public:
// constructor
 	Link(double alfa, double a, double d, double offset, double gamma, string path);
// methods
	Eigen::Matrix4d DenavitHartenbergMatrix(double theta);
// getters
	Eigen::Matrix4d* GetCumulatedMatrix() { return &(this->mCumulatedMatrix); };
	vector<Triangle>* GetTriangularMesh() { return &(this->mTriangularMesh); };
	vector<Triangle>* GetUpdatedTriangularMesh() { return &(this->mUpdatedTriangularMesh); };
	JacobMatrix* GetPosJacobian() { return &(this->mPosJacobian); };
	JacobMatrix* GetOriJacobian() { return &(this->mOriJacobian); };
	double GetGamma() { return this->mGamma; };
// setters
	void SetCumulatedMatrix(Eigen::Matrix4d A) { this->mCumulatedMatrix = A; };
	void SetPosJacobian(JacobMatrix jacob) { this->mPosJacobian = jacob; };
	void SetOriJacobian(JacobMatrix jacob) { this->mOriJacobian = jacob; };

//methods
	void UpdateTriangularMesh(vector<Triangle> new_mesh){ this->mUpdatedTriangularMesh = new_mesh; };

protected:
// data
    double mAlfa;
	double mA;
	double mD;
	double mOffset;
	double mGamma; // coefficient multiplying CSF differently for each link
private:
	Eigen::Matrix4d mCumulatedMatrix;
	JacobMatrix mPosJacobian;
	JacobMatrix mOriJacobian;
	vector<Triangle> mTriangularMesh; // in local DH frame
	vector<Triangle> mUpdatedTriangularMesh;
	
};

class RobotGripper : public Link {
	private:
		// //Eigen::Matrix4d mRelativeDisplacement;
		// bool mIsGripperClosed;

	public:
		// RobotGripper(bool isClosed, double alfa, double a, double d, double offset, double gamma, string path): 
		// 				Link(alfa,a,d,offset,gamma,path);
	
		// // getter
		// bool IsOpen() { return this->mIsGripperClosed; };
		// // setters
		// void SetClosed() { this->mIsGripperClosed = true; };
    	// void SetOpen() { this->mIsGripperClosed = false; };

	using Link::Link;

};

class Robot {

public:
// constructor
	Robot(ros::NodeHandle& input);
	
	void Spinner(void);

// attributes
	double csf;

	vector<Triangle> robot_mesh;

// methods
	int nDOF() { return (int) this->mLinks.size(); };
	void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void UpdateRobotMesh();
	void ComputeCoefficients();
	void ComputeSafetyInd();

// getters
	// Eigen::Matrix4d* GetBase() { return this->pBase; };
	Eigen::Matrix4d GetBase() { return this->pBase; };
	Eigen::VectorXd GetRobotPosition() { return (this->mJointPos); };
	Link* GetLinkAt(unsigned int i);
	Eigen::Matrix4d* GetCumulatedMatrix(unsigned int i) { return this->GetLinkAt(i)->GetCumulatedMatrix(); };
    vector<Triangle>* GetTriangularMesh() { return this->mBaseTriangularMesh; };
    JacobMatrix* GetPosJacobian(unsigned int i) { return this->GetLinkAt(i)->GetPosJacobian(); };
	JacobMatrix* GetOriJacobian(unsigned int i) { return this->GetLinkAt(i)->GetOriJacobian(); };

// setters
	void SetRobotState(double pos[6]);

// publishers
	void CsfPublisher();

	moveit::planning_interface::MoveGroupInterface* move_group_interface;
	moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
	const robot_state::JointModelGroup* joints;
	moveit::core::RobotState* robotState;
    // robot_state::RobotState* robotState;
	// const moveit::core::RobotState* other;
	void MoveRobot(geometry_msgs::Pose target_pose, bool enable_execution = false);
	void MoveRobot(sensor_msgs::JointState target_joints, bool enable_execution = false);

private:
// ROS handling
	ros::NodeHandle nh;
	ros::Subscriber sub_joint_state;
	ros::Publisher pointcloud;
	ros::Publisher pointcloud_mesh;
	sensor_msgs::PointCloud pc;
	sensor_msgs::PointCloud mesh_points;
	ros::Publisher pub_csf;
	std_msgs::Float64 msg_csf;

// data
    Eigen::Matrix4d pBase;
	Eigen::VectorXd mJointPos;
	Eigen::VectorXd mJointVel;
	list<Link> mLinks;
	vector<Triangle>* mBaseTriangularMesh; // mesh of the robot base in local base frame

	list<vector<double>> c_0, c_x, c_y;
	list<vector<Eigen::Vector3d>> ref_origins;

};

#endif
