#ifndef __H_INCLUDE_OBJECT__
#define __H_INCLUDE_OBJECT__


#include "general.h"
#include "camera.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "json.h"


struct Grasp
{
	Eigen::Matrix3d tool_orientation;
	Eigen::Vector3d tool_position;
	Eigen::Vector3d tool_position_offset;

	// --- Init Planning --- //
	double F1 = 999;
	double F2 = 999;
	double F3 = 999;
	double F4 = 999;
    double F5 = 0;
	double VolDx = 0;
	double VolSx = 0;

	int idx = -1;

	std::vector<Grasp*> nearGrasps = {};

};

class Object{
private:
    // Data
	CameraManager* camera;
    std::vector<Grasp> GraspsList;
    Eigen::Vector3d objPosEig;
    Eigen::Matrix3d objOrEig;
	geometry_msgs::Pose objPoseRos; 
	sensor_msgs::PointCloud voxelModel;
	sensor_msgs::PointCloud modelPointCloud;
	cv::Matx44d _objectPose;
	Eigen::Matrix4d objectPose;
	// Dimensions
	Eigen::Vector3d xyz_min;
	Eigen::Vector3d xyz_max;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener* tfListener;
	sensor_msgs::PointCloud gripperFrameVoxel;

	// aruco settings
	double object_aruco_dimension;
	int aruco_id_object;
	cv::Ptr<cv::aruco::Dictionary> objectDictionary;
	cv::Ptr<cv::aruco::DetectorParameters> pDetectorParams;

    int _V0;
	bool _tracked;

	// Setter
	void SetPose(Eigen::Matrix4d in) { this->objectPose = in; 
									   this->objOrEig = in.block<3,3>(0,0);
									   this->objPosEig = in.block<3,1>(0,3);
								//    this->objOrEig << this->_objectPose(0, 0), this->_objectPose(0, 1), this->_objectPose(0, 2), 
							   	// 					 this->_objectPose(1, 0), this->_objectPose(1, 1), this->_objectPose(1, 2),
							   	// 					 this->_objectPose(2, 0), this->_objectPose(2, 1), this->_objectPose(2, 2);
								//    this->objPosEig << this->_objectPose(0, 3), this->_objectPose(1, 3), this->_objectPose(2, 3);
	};
	// void SetPoseRos(geometry_msgs::Pose objectPoseDepthFrame){ this->objectPose = objectPoseDepthFrame; };

	// ROS
	ros::NodeHandle nh;
	tf2_ros::TransformBroadcaster objectFrameTf;

public:
    // Getters
	std::vector<Grasp>* GetGraspsList() { return &this->GraspsList; };
    Eigen::Vector3d GetObjPosition() { return this->objPosEig; };
	Eigen::Matrix3d GetObjOrientation() { return this->objOrEig; };
	// geometry_msgs::Pose GetPose(){ return this->objectPose; };
	sensor_msgs::PointCloud* GetVoxelModel() { return &this->voxelModel; };
	int GetV0() { return this->_V0; };

    // Constructor 
    Object(ros::NodeHandle nh);

    // Methods
	// void PoseTfTransform(std::string parent_frame, geometry_msgs::Pose gripperPoseNewFrame);
	// void PointTfTransform(std::string parent_frame, geometry_msgs::Point pointNewFrame);
	void PointcloudTfTransform(std::string target_frame, sensor_msgs::PointCloud pclInNewFrame);
	// geometry_msgs::Pose EigenToRosPose(Eigen::Matrix4d poseMatrix);
	// geometry_msgs::Quaternion QuaternionProduct(geometry_msgs::Quaternion arg1, Eigen::Quaterniond arg2);

	void ConnectNearGrasps();

	// Object tracking
	void Tracking();
	void setPose2Robot(Eigen::Matrix4d& tag2rgb);
    


    ros::Publisher pub;
	void Spinner();
};

#endif