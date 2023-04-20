#include "../include/object.hpp"

Object::Object(ros::NodeHandle input){
    
    this->nh = input;

    this->camera = new CameraManager(this->nh);

    // tf listener for function "TfTransform"
    this->tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Object Aruco settings + Aruco parameters
    this->object_aruco_dimension = 0.04;
    this->aruco_id_object = 40;
    this->objectDictionary = cv::aruco::getPredefinedDictionary(10); // 6x6_250  
    this->pDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
    this->pDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    this->pDetectorParams->cornerRefinementMaxIterations = 50;
    this->pDetectorParams->cornerRefinementMinAccuracy = 0.05;
    this->pDetectorParams->maxErroneousBitsInBorderRate = 0.0;

    // Loading grasp list
    // std::ifstream file("../include/cube/GraspsList.json");
    std::ifstream file("src/robot/include/cube/GraspsList.json");
	Json::Value root;
	file >> root;

    for (int i = 0; i < root["list"].size(); i++) // for every grasp in grasps list
	{
		Grasp g;

		g.tool_orientation << root["list"][i]["pose"][0].asDouble(), root["list"][i]["pose"][1].asDouble(), root["list"][i]["pose"][2].asDouble(),
						      root["list"][i]["pose"][4].asDouble(), root["list"][i]["pose"][5].asDouble(), root["list"][i]["pose"][6].asDouble(),
							  root["list"][i]["pose"][8].asDouble(), root["list"][i]["pose"][9].asDouble(), root["list"][i]["pose"][10].asDouble();

		g.tool_position << root["list"][i]["pose"][3].asDouble(), root["list"][i]["pose"][7].asDouble(), root["list"][i]["pose"][11].asDouble();

		g.tool_position_offset = g.tool_position + g.tool_orientation.block<3, 1>(0, 0) * FINGER_OFFSET;
		
		g.F1 = root["list"][i]["value"].asDouble();
		g.idx = i;
		GraspsList.push_back(g);
	}
	file.close();

    // Loading voxel model 
    // std::string name = "cube";
    // cv::String path = "../include";
	std::string line;
	Eigen::Vector3d row;
    // std::ifstream file(path + name + "/GraspsList.json");

    // file.open("../include/cube/Voxel.txt");
    file.open("src/robot/include/cube/Voxel.txt");
	// file.open(path + name + "/Voxel.txt");

	if (!file) {
		std::cerr << "Unable to open file datafile.txt";
		exit(1);   // call system to stop
	}

    this->voxelModel.header.stamp = ros::Time::now();
    this->voxelModel.header.frame_id = "object_frame";
    // this->voxelModel.header.frame_id = "world";

    unsigned int j=0;
	while (std::getline(file, line)) // fill pcl
	{
		for (int i = 0; i < 3; i++)
			file >> row(i);

        geometry_msgs::Point32 point; 
        point.x = row(0) * 0.001 ; point.y = row(1) * 0.001 ; point.z = row(2) * 0.001 ;

        this->voxelModel.points.push_back(point);

        j++;
	}
	file.close();

	this->pub = this->nh.advertise<sensor_msgs::PointCloud>("/voxeltopic", 1);


}

// Function performing the object tracking detecting its aruco marker.
// The object is returned as a pointcloud, referred to its own frame (obtained by the voxel model),
// which is also connected to the tf tree 
void Object::Tracking()
{	
	//  ARUCO tracking
    std::vector<int> found_id_object;
    std::vector<std::vector<cv::Point2f>> found_corners_object;

    geometry_msgs::TransformStamped transformWorldToRgb = this->tfBuffer.lookupTransform("world", "rgb_camera_link", ros::Time(0), ros::Duration(1));
// qua mi piacerebbe usare la funzione camera->GetCvRgbImage
    sensor_msgs::CompressedImageConstPtr p_camera_image = ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/rgb/image_raw/compressed", ros::Duration(30000));

    cv::Mat cvIm;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*p_camera_image, "bgr8");
    cv_ptr->image.copyTo(cvIm);

    cv::aruco::detectMarkers(cvIm, this->objectDictionary, found_corners_object, found_id_object, this->pDetectorParams);

    std::vector<cv::Vec3d> rvec, tvec;
    cv::aruco::estimatePoseSingleMarkers(found_corners_object, this->object_aruco_dimension, *camera->GetRgbCameraMatrix(), *camera->GetRgbDistortionCoeffs(), rvec, tvec);

    cv::Mat R(3, 3, CV_64FC1);
    cv::Rodrigues(rvec.at(0), R);

    Eigen::Matrix4d T_cameraRgb_objectAruco;
    T_cameraRgb_objectAruco << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at(0)[0],
                               R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at(0)[1],
                               R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at(0)[2],
                                              0.0,                0.0,                0.0,           1.0;

    Eigen::Matrix4d T_world_cameraRgb;
    T_world_cameraRgb.block<3, 1>(0, 3) = Eigen::Vector3d( transformWorldToRgb.transform.translation.x, 
                                                           transformWorldToRgb.transform.translation.y, 
                                                           transformWorldToRgb.transform.translation.z );
    T_world_cameraRgb.block<3, 3>(0, 0) = Eigen::Matrix3d( Eigen::Quaterniond( transformWorldToRgb.transform.rotation.w, 
                                                                               transformWorldToRgb.transform.rotation.x, 
                                                                               transformWorldToRgb.transform.rotation.y, 
                                                                               transformWorldToRgb.transform.rotation.z ).toRotationMatrix() );

/*
    // geometry_msgs::TransformStamped transformStamped;
    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "rgb_camera_link";
    // transformStamped.child_frame_id = "object_aruco";

    // Eigen::Vector3d    position = Eigen::Vector3d( T_cameraRgb_objectAruco.block<3, 1>(0, 3) );
    // Eigen::Quaterniond orientation = Eigen::Quaterniond( T_cameraRgb_objectAruco.block<3, 3>(0, 0) );

    // transformStamped.transform.rotation.w = orientation.w();
    // transformStamped.transform.rotation.x = orientation.x();
    // transformStamped.transform.rotation.y = orientation.y();
    // transformStamped.transform.rotation.z = orientation.z();

    // ROS_INFO("%lf, %lf, %lf, %lf", orientation.w(), orientation.x(), orientation.y(), position.x());

    // // Translation
    // transformStamped.transform.translation.x = position.x();
    // transformStamped.transform.translation.y = position.y();
    // transformStamped.transform.translation.z = position.z();

    // // Writing the tfs
    // this->objectFrameTf.sendTransform(transformStamped);

*/

    Eigen::Matrix4d T_objAruco_objReference;

    T_objAruco_objReference <<  0.0, 0.0,  1.0,   0.0,
                                0.0, 1.0,  0.0,   0.0,
                               -1.0, 0.0,  0.0, -0.03,
                                0.0, 0.0,  0.0,   1.0;


    // T_objAruco_objReference << ...

    Eigen::Matrix4d T_cameraRgb_objReference = T_cameraRgb_objectAruco * T_objAruco_objReference;
    Eigen::Matrix4d T_world_objectRef = T_world_cameraRgb * T_cameraRgb_objReference;
    
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "object_frame";

    Eigen::Vector3d    position = Eigen::Vector3d( T_world_objectRef.block<3, 1>(0, 3) );
    Eigen::Quaterniond orientation = Eigen::Quaterniond( T_world_objectRef.block<3, 3>(0, 0) );
    // position = Eigen::Vector3d( T_world_objectRef.block<3, 1>(0, 3) );
    // orientation = Eigen::Quaterniond( T_world_objectRef.block<3, 3>(0, 0) );

    transformStamped.transform.rotation.w = orientation.w();
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();

    // Translation
    transformStamped.transform.translation.x = position.x();
    transformStamped.transform.translation.y = position.y();
    transformStamped.transform.translation.z = position.z();

    // Writing the tfs
    this->objectFrameTf.sendTransform(transformStamped);

    // geometry_msgs::Pose objPoseDepthFrame;
    // in << T_cameraBase_cameraRgb * T_cameraRgb_objectAruco; // questo va cambiato! Dice di prendere l'oggetto dall'aruco
    this->SetPose(T_cameraRgb_objReference);

    // this->SetPoseRos(objPoseDepthFrame);

}

// Function to easily transform voxel model pointcloud in any reference frame.
// Input: target frame (I want the pointcloud referred to this frame); output: pointcloud referred to desired frame (pclInNewFrame)
void Object::PointcloudTfTransform(std::string target_frame, sensor_msgs::PointCloud pclInNewFrame){

	geometry_msgs::TransformStamped targetToObjTf =  this->tfBuffer.lookupTransform(target_frame, "object_frame", ros::Time(0), ros::Duration(1));

    geometry_msgs::PointStamped stampedVoxelModel;
    geometry_msgs::PointStamped pointTargetToObj;

    for(unsigned int voxel_i; voxel_i < this->voxelModel.points.size(); voxel_i++){
        stampedVoxelModel.point.x = this->voxelModel.points.at(voxel_i).x;
        stampedVoxelModel.point.y = this->voxelModel.points.at(voxel_i).y;
        stampedVoxelModel.point.z = this->voxelModel.points.at(voxel_i).z;

        tf2::doTransform(stampedVoxelModel, pointTargetToObj, targetToObjTf);

        pclInNewFrame.points.at(voxel_i).x = pointTargetToObj.point.x;
        pclInNewFrame.points.at(voxel_i).y = pointTargetToObj.point.y;
        pclInNewFrame.points.at(voxel_i).z = pointTargetToObj.point.z;

    }

}
// void Object::PoseTfTransform(std::string parent_frame, geometry_msgs::Pose poseNewFrame){

// 	geometry_msgs::TransformStamped parentToObjTf =  this->tfBuffer.lookupTransform(parent_frame, "object_frame", ros::Time(0), ros::Duration(1));

//     geometry_msgs::PoseStamped stampedPose;
//     stampedPose.pose = poseNewFrame;
//     geometry_msgs::PoseStamped newPoseStamped;
    
//     tf2::doTransform(stampedPose, newPoseStamped, parentToObjTf);

//     poseNewFrame = newPoseStamped.pose;

// }
// void Object::PointTfTransform(std::string parent_frame, geometry_msgs::Point pointNewFrame){

// 	geometry_msgs::TransformStamped parentToObjTf =  this->tfBuffer.lookupTransform(parent_frame, "object_frame", ros::Time(0), ros::Duration(1));

//     geometry_msgs::PointStamped stampedPoint;
//     stampedPoint.point = pointNewFrame;
//     geometry_msgs::PointStamped newPointStamped;

//     tf2::doTransform(stampedPoint, newPointStamped, parentToObjTf);

//     pointNewFrame = newPointStamped.point;

// }

// geometry_msgs::Quaternion Object::QuaternionProduct(geometry_msgs::Quaternion arg1, Eigen::Quaterniond arg2){
//     geometry_msgs::Quaternion out;
//     out.x = arg1.w * arg2.x() + arg1.x * arg2.w() + arg1.y * arg2.z() - arg1.z * arg2.y();
//     out.y = arg1.w * arg2.y() + arg1.y * arg2.w() + arg1.z * arg2.x() - arg1.x * arg2.z();
//     out.z = arg1.w * arg2.z() + arg1.z * arg2.w() + arg1.x * arg2.y() - arg1.y * arg2.x();
//     out.w = arg1.w * arg2.w() - arg1.x * arg2.x() - arg1.y * arg2.y() - arg1.z * arg2.z();
//     return out;
// }

void Object::ConnectNearGrasps(){


	Eigen::Vector3d approach_i;
	Eigen::Vector3d approach_j;

	for (int i = 0; i < this->GraspsList.size(); i++)
	{
		this->GraspsList[i].nearGrasps.push_back(&this->GraspsList[i]); // aggiunge se stesso

		approach_i << this->GraspsList[i].tool_orientation(0, 0), this->GraspsList[i].tool_orientation(1, 0), this->GraspsList[i].tool_orientation(2, 0);
		for (int j = 0; j < this->GraspsList.size(); j++)
		{
			approach_j << this->GraspsList[j].tool_orientation(0, 0), this->GraspsList[j].tool_orientation(1, 0), this->GraspsList[j].tool_orientation(2, 0);
			double coseno = (approach_i[0] * approach_j[0] + approach_i[1] * approach_j[1] + approach_i[2] * approach_j[2]) / (approach_i.norm() * approach_j.norm());

			// condizione = angolo tra gli approcci (abs) < 30� (step angle) <-> cos dell'angolo tra i due vettori compreso tra 1 e 0.86
			// e distanza tra tcp < 2 cm (valore ragionevole?)   
			if ( coseno > 0.3 && coseno <= 1 && (this->GraspsList[i].tool_position - this->GraspsList[j].tool_position).norm() < 10 && i!=j)
				this->GraspsList[i].nearGrasps.push_back(&this->GraspsList[j]);
		}
	}

}

// void Object::setPose2Robot(Eigen::Matrix4d& tag2rgb)
void Object::setPose2Robot(Eigen::Matrix4d& tag2rgb)
{

	double alpha = -6 * PIGR / 180;
	// cv::Matx44d rgb2depth(1, 0, 0, -32.0, 0, cos(alpha), -sin(alpha), 0, 0, sin(alpha), cos(alpha), 0, 0, 0, 0, 1.0);
	// cv::Matx44d robot2tag(1, 0, 0, -135.0, 0, 1, 0, -10.0, 0, 0, 1, -30.0, 0, 0, 0, 1.0); // per cilindro in verticale riduco Z a -50, normalmente -30
	Eigen::Matrix4d robot2tag;
    robot2tag << 1.0, 0.0, 0.0, -135.0,
                 0.0, 1.0, 0.0,  -10.0,
                 0.0, 0.0, 1.0,  -30.0,
                 0.0, 0.0, 0.0,    1.0; // per cilindro in verticale riduco Z a -50, normalmente -30

	// x aumenta -> pinza si sapsta verso +x
    // y aumenta -> pinza si sposta verso +y
    // z aumenta -> pinza si sposta in alto (+z)

	// cv::Matx44d rob2obj = robot2tag * tag2rgb * rgb2depth * this->_objectPose;
	Eigen::Matrix4d rob2obj = robot2tag * tag2rgb * this->objectPose; // this objectPose (set by function Pose) is referred to rgb camera

    this->objOrEig = rob2obj.block<3, 3>(0,0);
    this->objPosEig = rob2obj.block<3, 1>(0,3);


    // geometry_msgs::Pose objectRobFrame = objectPose;
    // this->PoseTfTransform("base_0", objectRobFrame);
    
    // this->objectPose = objectRobFrame;

	// std::cout << "\nRefined pose wrt TAG: \n" << "Rotation matrix \n" << this->obj_R_eigen << std::endl << "Position \n" << this->obj_p_eigen << std::endl; // print refined pose
}

void Object::Spinner(){
    // sensor_msgs::PointCloud msg;
    // msg.points = this->GetVoxelModel()->points;
	// this->pub.publish(msg);
    this->pub.publish(*this->GetVoxelModel());
}