#include "../include/grasp_planning.hpp"

GraspPlanner::GraspPlanner(ros::NodeHandle input){

	this->nh = input;

	this->pRobot = new Robot(this->nh);

	// this->move_group = new moveit::planning_interface::MoveGroupInterface("arm");

	// va messa come variabile della classe?
	this->joint_group = this->pRobot->move_group_interface->getCurrentState()->getJointModelGroup("arm"); 
	
	this->RagagliaCoefficients(this->pRobot);

}

void GraspPlanner::GraspsRanking(Object* pObj){

	this->sortedGraspList = std::vector<Grasp>();
	int graspCounter = 1;

	for (Grasp& eachGrasp : *(pObj->GetGraspsList()))
	{
		Eigen::Matrix4d gripperPose = this->GetGripperPose(pObj, &eachGrasp);
        
		// --- IKIN --- //

		bool WeGotJoints = this->pRobot->robotState->setFromIK(this->joint_group, this->EigenToRosPose(gripperPose));
		Eigen::VectorXd jointValues;

		this->pRobot->robotState->copyJointGroupPositions(this->joint_group, jointValues);
		ROS_INFO("got them? %d \n %lf, %lf, %lf, %lf, %lf, %lf", WeGotJoints, jointValues(0), jointValues(1), jointValues(2), jointValues(3), jointValues(4), jointValues(5));

		if (WeGotJoints) // If true -> compute F3
		{
			ROS_INFO("1");
			eachGrasp.F2 = AxisSelection(pObj, eachGrasp, gripperPose);
			ROS_INFO("2");
			// eachGrasp.F3 = ComputeF3(this->pRobot->joints);
			eachGrasp.F3 = ComputeF3(jointValues);
			ROS_INFO("3");
			eachGrasp.F4 = ComputeF4_OnApproach(gripperPose);
			ROS_INFO("4");
			// eachGrasp.F4 = ComputeF4(this->pRobot->joints);
			eachGrasp.F4 = ComputeF4(jointValues);
			ROS_INFO("5");
			eachGrasp.F5 = ComputeF5(this->pRobot);
			ROS_INFO("6");
			this->sortedGraspList.push_back(eachGrasp);

			ROS_INFO("7");
		}

		graspCounter++;
		
	}

	std::sort(this->sortedGraspList.begin(), this->sortedGraspList.end(), [](const Grasp& lhs, const Grasp& rhs)
	{
		return lhs.F1 + lhs.F2 + lhs.F3 + lhs.F4 < rhs.F1 + rhs.F2 + rhs.F3 + rhs.F4;
	});

}

void GraspPlanner::NearGraspsRanking(Object* pObj, int& bestGraspIdx){
	
	this->sortedGraspList = std::vector<Grasp>();
	int graspCounter = 1;

	for (Grasp* eachPGrasp : pObj->GetGraspsList()[0][bestGraspIdx].nearGrasps){

		// std::cout << eachPGrasp->idx + 1 << std::endl;
		Eigen::Matrix4d nearGripperPose = this->GetGripperPose(pObj, eachPGrasp);

		const robot_state::JointModelGroup* near_joint_group = this->pRobot->move_group_interface->getCurrentState()->getJointModelGroup("arm");
		
		bool WeGotJoints = this->pRobot->robotState->setFromIK(near_joint_group, this->EigenToRosPose(nearGripperPose));
		Eigen::VectorXd nearJointValues;
		this->pRobot->robotState->copyJointGroupPositions(near_joint_group, nearJointValues);
		
		// --- IKIN --- //
		// const robot_state::JointModelGroup* nearJoints = this->pRobot->move_group->getCurrentState()->getJointModelGroup("arm");
		// const robot_state::JointModelGroup* nearJoints = this->move_group->getCurrentState()->getJointModelGroup("arm");

		// bool WeGotJoints = this->pRobot->robotState->setFromIK(nearJoints, this->EigenToRosPose(nearGripperPose));
		// bool WeGotJoints = this->robotState->setFromIK(nearJoints, this->EigenToRosPose(nearGripperPose));
		// std::cout << " Required time for Ikin = "<< ((double)(tick4 - tick3) / cv::getTickFrequency()) << " seconds" << std::endl;; // tempo totale per calcolare ikin
		
		if (WeGotJoints) // If true -> compute F3
		{
			eachPGrasp->F2 = AxisSelection(pObj, *eachPGrasp, nearGripperPose);
			eachPGrasp->F3 = ComputeF3(nearJointValues);
			// eachGrasp->F4 = ComputeF4(joints);
			eachPGrasp->F4 = ComputeF4_OnApproach(nearGripperPose);
			// Print Best Grasp
			this->sortedGraspList.push_back(*eachPGrasp);
		}

		graspCounter++;
	}

	std::sort(this->sortedGraspList.begin(), this->sortedGraspList.end(), [](const Grasp& lhs, const Grasp& rhs)
		{
			return lhs.F1 + lhs.F2 + lhs.F3 + lhs.F4 < rhs.F1 + rhs.F2 + rhs.F3 + rhs.F4;
		});

	//std::cout << " Time required for grasp planning =  " <<(double)(tick2 - tick1) / cv::getTickFrequency() << " seconds \n" ;

}

void GraspPlanner::BestGraspJoints(Object* pObj, Eigen::VectorXd approachJointValues, int& bestGraspIdx){

	for (int graspIdx = 0; graspIdx < this->sortedGraspList.size(); graspIdx++)
	{
		Eigen::Matrix4d gripperPose = GetGripperPose(pObj, &this->sortedGraspList[graspIdx]);

		const robot_state::JointModelGroup* approach_joint_group = this->pRobot->move_group_interface->getCurrentState()->getJointModelGroup("arm");

		// This IK should be with collision avoidance!!!
		// bool WeGotJoints = this->pRobot->robotState->setFromIK(approachJoints, this->EigenToRosPose(gripperPose));
		bool WeGotJoints = this->pRobot->robotState->setFromIK(approach_joint_group, this->EigenToRosPose(gripperPose));
		this->pRobot->robotState->copyJointGroupPositions(approach_joint_group, approachJointValues);

		if (WeGotJoints)
		{
			bestGraspIdx = this->sortedGraspList[graspIdx].idx;
		}
	}

	bestGraspIdx = -1;

}

Eigen::Matrix4d GraspPlanner::GetGripperPose(Object* obj, Grasp* grasp){
	
	Eigen::Vector3d sliding;
	sliding << grasp->tool_orientation(0, 1), grasp->tool_orientation(1, 1), grasp->tool_orientation(2, 1);

	
	// geometry_msgs::Pose gripperPose = obj->GetPose();
	// Eigen::Quaterniond gripperOrQuat;
	// gripperOrQuat.x() = gripperPose.orientation.x;
	// gripperOrQuat.y() = gripperPose.orientation.y;
	// gripperOrQuat.z() = gripperPose.orientation.z;
	// gripperOrQuat.w() = gripperPose.orientation.w;
	// Eigen::Matrix3d gripperOrMat = gripperOrQuat.toRotationMatrix();
	// Eigen::Vector3d offsetRobFrame = gripperOrMat * grasp->tool_position_offset;
	// geometry_msgs::Pose gripperPoseFinal;
	// gripperPoseFinal.position.x = gripperPose.position.x + offsetRobFrame.x();
	// gripperPoseFinal.position.y = gripperPose.position.y + offsetRobFrame.y();
	// gripperPoseFinal.position.z = gripperPose.position.z + offsetRobFrame.z();
	// Eigen::Quaternion<double> rot_frame_quat = Eigen::Quaternion<double>(Eigen::AngleAxisd( PIGR / 2, sliding ));
	// Eigen::Quaternion<double> toolOrientation = Eigen::Quaternion<double>(grasp->tool_orientation);
	// gripperPoseFinal.orientation = obj->QuaternionProduct(gripperPose.orientation, rot_frame_quat);
	// gripperPoseFinal.orientation = obj->QuaternionProduct(gripperPoseFinal.orientation, toolOrientation);
	
	Eigen::Vector3d gripperPosEig = obj->GetObjPosition() + obj->GetObjOrientation() * grasp->tool_position_offset;

	Eigen::Matrix3d rot_frame; 
	rot_frame = Eigen::AngleAxisd( PIGR / 2, sliding );

	Eigen::Matrix3d gripperOrEig = obj->GetObjOrientation() * rot_frame * grasp->tool_orientation; // orientamento della flangia

	Eigen::Vector3d euler_tcp_orientation_r = gripperOrEig.eulerAngles(2, 1, 2);
	gripperOrEig = Eigen::AngleAxisd(euler_tcp_orientation_r[0], Eigen::Vector3d::UnitZ()) * 
				   Eigen::AngleAxisd(euler_tcp_orientation_r[1], Eigen::Vector3d::UnitY()) *
				   Eigen::AngleAxisd(euler_tcp_orientation_r[2], Eigen::Vector3d::UnitZ());

	Eigen::Matrix4d gripperPoseEig;
	gripperPoseEig << gripperOrEig(0, 0), gripperOrEig(0, 1), gripperOrEig(0, 2), gripperPosEig(0),
					  gripperOrEig(1, 0), gripperOrEig(1, 1), gripperOrEig(1, 2), gripperPosEig(1),
					  gripperOrEig(2, 0), gripperOrEig(2, 1), gripperOrEig(2, 2), gripperPosEig(2),
					  				 0.0,				 0.0,			  	 0.0,			   1.0;

	// geometry_msgs::Pose gripperPoseRos;
	// gripperPoseRos.position.x = gripperPosEig(0); 
	// gripperPoseRos.position.y = gripperPosEig(1); 
	// gripperPoseRos.position.z = gripperPosEig(2);

	// Eigen::Quaternion<double> gripperQuat = Eigen::Quaternion<double>(gripperOrEig);
	// gripperPoseRos.orientation.w = gripperQuat.w();
	// gripperPoseRos.orientation.x = gripperQuat.x();
	// gripperPoseRos.orientation.y = gripperQuat.y();
	// gripperPoseRos.orientation.z = gripperQuat.z();
	
	// geometry_msgs::Point position = gripperPoseRos.position;
	// geometry_msgs::Quaternion orientation = gripperPoseRos.orientation;
		
	// ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf",
	// position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z);


	return gripperPoseEig;
}

double GraspPlanner::AxisSelection(Object* pObj, Grasp grasp, Eigen::Matrix4d gripperPose){

	// cv::Matx44d* objPose = pObj->GetBestPose();
	Eigen::Vector3d v;
	Eigen::Vector3d center;

	Eigen::Vector3d position;
	position(0) = gripperPose(0, 3);
	position(1) = gripperPose(1, 3);
	position(2) = gripperPose(2, 3);
	
	center << gripperPose(0, 3), gripperPose(1, 3), gripperPose(2, 3);

	// std::cout << "Center: \n" << center << std::endl;

	double f2 = 0;
    if (gripperPose(0, 2) >= -0.0001 && gripperPose(0, 2) <= 0.0001 && gripperPose(1, 2) >= -0.0001 && gripperPose(1, 2) <= 0.0001) // approccio perfettamente verticale dall'alto o dal basso 
    {
		v << gripperPose(0, 1), gripperPose(1, 1), gripperPose(2, 1); // normal direction
		f2 = this->ComputeF2(pObj, grasp, v, center);
    }
        
    else if ( ( gripperPose(2, 2) >= -0.0001 && gripperPose(2, 2) <= 0.0001) // approccio perfettamente orizzontale
		   && ( gripperPose(0, 0) >= -0.0001 && gripperPose(0, 0) <= 0.0001 && gripperPose(1, 0) >= -0.0001 && gripperPose(1, 0) <= 0.0001) ) // con pinza con facce parallele al terreno

    {
		v << gripperPose(0, 1), gripperPose(1, 1), gripperPose(2, 1); // normal direction
		f2 = this->ComputeF2(pObj, grasp, v, center);
    }

	else if ( ( gripperPose(2, 2) >= -0.0001 && gripperPose(2, 2) <= 0.0001) // approccio perfettamente orizzontale
		   && ( gripperPose(2, 0) >= -0.0001 && gripperPose(2, 0) <= 0.0001) ) // con pinza con facce perpendicolari al terreno
	{
		v << gripperPose(0, 2), gripperPose(1, 2), gripperPose(2, 2); // approach direction
		f2 = this->ComputeF2(pObj, grasp, v, center);
	}

    else // qualunque altro approccio, conta solo componente sul piano xy
    {
        v << gripperPose(0, 1), gripperPose(1, 1), 0;
		f2 = this->ComputeF2(pObj, grasp, v, center);
    }

	return f2;
}

double GraspPlanner::ComputeF2(Object* pObj, Grasp grasp, Eigen::Vector3d vector, Eigen::Vector3d center){

	grasp.VolDx = 0;
	grasp.VolSx = 0;

	double f2; 
    VolumeCount(pObj, grasp, vector, center);

    if (grasp.VolSx == 0 || grasp.VolDx == 0)
		f2 = 999;

    else if (grasp.VolDx >= grasp.VolSx)
		f2 = grasp.VolDx / grasp.VolSx;
    
    else if (grasp.VolDx <= grasp.VolSx)
		f2 = grasp.VolSx / grasp.VolDx;

	return f2;

}

double GraspPlanner::ComputeF3(Eigen::VectorXd jointValues){

	// Eigen::Matrix<double, 3, Eigen::Dynamic>* posJacobian = this->pRobot->GetPosJacobian(6);
	// Eigen::Matrix<double, 3, Eigen::Dynamic>* orJacobian = this->pRobot->GetOriJacobian(6);
	Eigen::Matrix<double, 3, Eigen::Dynamic>* posJacobian = this->pRobot->GetPosJacobian(6);
	Eigen::Matrix<double, 3, Eigen::Dynamic>* orJacobian = this->pRobot->GetOriJacobian(6);

	Eigen::Matrix<double, 6, 6> jacobian;
	jacobian.block<3, 6>(0,0) = *posJacobian;
	jacobian.block<3, 6>(3,0) = *orJacobian;

    if (jacobian.determinant() != 0)
    {
        Eigen::Matrix<double, 6, 6> prod = jacobian * jacobian.transpose();
        return 1 / sqrt(prod.determinant());
    }
    else
        return 999;

}

double GraspPlanner::ComputeF4_OnApproach(Eigen::Matrix4d gripperPose){

	Eigen::Matrix4d approachPose = gripperPose;

	approachPose(0, 3) -= (APPROACH_OFFSET * gripperPose(0, 2));
	approachPose(1, 3) -= (APPROACH_OFFSET * gripperPose(1, 2));
	approachPose(2, 3) -= (APPROACH_OFFSET * gripperPose(2, 2));

	const robot_state::JointModelGroup* approach_joint_group = this->pRobot->move_group_interface->getCurrentState()->getJointModelGroup("arm");
	bool WeGotJoints = this->pRobot->robotState->setFromIK(approach_joint_group, this->EigenToRosPose(gripperPose));
	Eigen::VectorXd approachJointValues;
	this->pRobot->robotState->copyJointGroupPositions(approach_joint_group, approachJointValues);

	// const robot_state::JointModelGroup* approachJoints = this->pRobot->move_group->getCurrentState()->getJointModelGroup("arm");
	// const robot_state::JointModelGroup* approachJoints = this->move_group->getCurrentState()->getJointModelGroup("arm");
	// this->pRobot->robotState->setFromIK(approachJoints, this->EigenToRosPose(approachPose));
	// this->robotState->setFromIK(approachJoints, this->EigenToRosPose(approachPose));

	// "link0" must be replaced in order to scan each link
	double abs_error = 0;
	for (int i = 0; i < approachJointValues.size(); i++)
		// abs_error += abs( *this->pRobot->move_group->getCurrentState()->getJointPositions("link0") - this->pRobot->GetRobotPosition()[i] );
		abs_error += abs( approachJointValues(i) - this->pRobot->GetRobotPosition()[i] );

	return abs_error / 6;
}

double GraspPlanner::ComputeF4(Eigen::VectorXd jointValues){ // these joints are obtained by inv kin

	double abs_error = 0;
	for (int i = 0; i < this->pRobot->GetRobotPosition().size(); i++)
		// abs_error += abs( *this->pRobot->move_group->getCurrentState()->getJointPositions("link0") - this->pRobot->GetRobotPosition()[i] );
		abs_error += abs( jointValues(i) - this->pRobot->GetRobotPosition()[i] );
		
	return abs_error / 6 ; // (6 = number of joints)

}

double GraspPlanner::ComputeF5(Robot* robot){

	unsigned int n_points = this->bodyPointcloud.points.size();

	std::vector<Triangle>::iterator it_t;
	std::list<std::vector<Eigen::Vector3d>>::iterator it_origins = this->ref_origins.begin();
	std::list<std::vector<double>>::iterator it_c0 = this->c_0.begin(), it_cx = this->c_x.begin(), it_cy = this->c_y.begin();
	Eigen::Vector3d * point_A, * point_B, *point_C;
	unsigned int i, j, k;

	Eigen::Vector3d dist_point; // distance of body point from origin of the triangle reference
	double dist_point_n;

	// Axes of the reference frames placed on each triangle
	Eigen::Vector3d x_axis, y_axis, z_axis;

	// distances of a body point along x and y axes
	double dist_x, dist_y; 

	double safetyIndex; // cumulative safety field

	for(i = 0; i < robot->nDOF(); ++i) {
		k=0;

		for(it_t = robot->GetLinkAt(i)->GetUpdatedTriangularMesh()->begin(); 
			it_t != robot->GetLinkAt(i)->GetUpdatedTriangularMesh()->end(); ++it_t) {

		// Define vertices and reference frame of the i-th triangle 
			point_A = it_t->GetPoint1();
			point_B = it_t->GetPoint2();
			point_C = it_t->GetPoint3();

			Eigen::Vector3d seg_AB = *point_B - *point_A;
			Eigen::Vector3d dir_AB = seg_AB / seg_AB.norm();
			Eigen::Vector3d seg_AC = *point_C - *point_A;

			double proj_len = seg_AC.dot(dir_AB); // length of segment AP

			Eigen::Vector3d ref_origin = *point_A + dir_AB * proj_len; // actual projection (point P)

			x_axis = (* point_C - ref_origin) / (*point_C - ref_origin).norm(); // x is defined as the direction from P to C
			y_axis = (* point_B - *point_A) / (*point_B - *point_A).norm(); // y is defined as the direction from A to B
			z_axis = x_axis.cross(y_axis);


			// Sometimes point C coincides with the origin of the reference, or two points
			// (A and B) of a triangle coincide. This IF allows to avoid errors due to these situations.
			if ((*point_C - ref_origin).norm() != 0 && (*point_B - *point_A).norm() != 0){

				// For each body point, compute its distance to the origin of the i-th triangle frame and increment CSF
				for (j = 0; j < n_points; j++) {
					Eigen::Vector3d pc;
					pc << this->bodyPointcloud.points.at(j).x, this->bodyPointcloud.points.at(j).y, this->bodyPointcloud.points.at(j).z;

					dist_point = pc - ref_origin;
					// dist_point = body_points[j] - ref_origin;
					dist_point_n = dist_point.norm();

					dist_x = dist_point.dot(x_axis);
					dist_y = dist_point.dot(y_axis);
	
					safetyIndex = safetyIndex + robot->GetLinkAt(i)->GetGamma() * 1e-4 *
										 	  ( it_c0->at(k) + it_cx->at(k) * dist_x  
										 	  + it_cy->at(k) * dist_y + dist_point_n * dist_point_n);

				}

			}
			k++;

		}

		it_origins++; it_c0++; it_cx++; it_cy++;

	}

	return safetyIndex;
ROS_INFO("csf: %lf", safetyIndex);

}

// Ragaglia coefficients are the result of an integral, they are used to compute the safety index
// and are constant wrt the robot configuration, so they can be computed just once
void GraspPlanner::RagagliaCoefficients(Robot* robot){

	std::vector<Triangle>::iterator it_t;
	Eigen::Vector3d * point_A, * point_B, * point_C; // the vertices of each triangle
	Eigen::Vector3d point_P;
	Eigen::Vector3d dir_AB, seg_AB, seg_AC;

	unsigned int i;

	for(i = 0; i < robot->nDOF(); ++i) {
		std::vector<double> coeff_0, coeff_x, coeff_y;
		std::vector<Eigen::Vector3d> frameOrigins;

		for(it_t = robot->GetLinkAt(i)->GetTriangularMesh()->begin(); it_t != robot->GetLinkAt(i)->GetTriangularMesh()->end(); ++it_t) {

			point_A = it_t->GetPoint1();
			point_B = it_t->GetPoint2();
			point_C = it_t->GetPoint3();

			seg_AB = *point_B - *point_A;
			dir_AB = seg_AB / seg_AB.norm();
			seg_AC = *point_C - *point_A;

			double proj_len = seg_AC.dot(dir_AB); // length of segment AP

			point_P = *point_A + dir_AB * proj_len; // actual projection (point P)
			frameOrigins.push_back(point_P); // the point P is the origin of the ref. frame fixed to the triangle
			
			Eigen::Vector3d seg_PC = *point_C - point_P;
			Eigen::Vector3d seg_PA = *point_A - point_P;

			double coord_xC = seg_PC.norm();
			double coord_yA = seg_PA.norm();

			if (proj_len > 0) {  // if the point P is inside segment AB (angle in A is acute)
				coord_yA = -coord_yA;
			}

			double coord_yB = seg_AB.norm() + coord_yA;

			// (sort of) Ragaglia's coefficients
			coeff_0.push_back(coord_xC * coord_xC / 6.0 + (coord_yB * coord_yB + coord_yB * coord_yA + coord_yA * coord_yA) / 6.0);
			coeff_x.push_back(-2.0 / 3.0 * coord_xC);
			coeff_y.push_back(-2.0 / 3.0 * (coord_yA + coord_yB));
			
		}
		
		// Ragaglia coefficients stored in lists of vectors (one vector for each link, 
		// each vector has same dimension as the number of mesh triangles)
		this->c_0.push_back(coeff_0);
		this->c_x.push_back(coeff_x);
		this->c_y.push_back(coeff_y);
		this->ref_origins.push_back(frameOrigins);
		
	}

}

void GraspPlanner::VolumeCount(Object* obj, Grasp& grasp, Eigen::Vector3d& vector, Eigen::Vector3d& center){

	Eigen::Matrix3d R = obj->GetObjOrientation();
	Eigen::Vector3d p = obj->GetObjPosition();

	sensor_msgs::PointCloud voxelModelGripperRef;
	obj->PointcloudTfTransform("gripper_doosan", voxelModelGripperRef);

    for (int i = 0; i < (*obj->GetVoxelModel()).points.size() - 1; i++)
    {
		Eigen::Vector3d eigPoint;
		// eigPoint(0) = (obj->GetVoxelModel()->points.at(i)).x;
		// eigPoint(1) = (obj->GetVoxelModel()->points.at(i)).y;
		// eigPoint(2) = (obj->GetVoxelModel()->points.at(i)).z;
		eigPoint(0) = voxelModelGripperRef.points.at(i).x;
		eigPoint(1) = voxelModelGripperRef.points.at(i).y;
		eigPoint(2) = voxelModelGripperRef.points.at(i).z;

        // Eigen::Vector3d point = R * eigPoint + p;  // usando le giuste tf questo passaggio può essere saltato (??)
        // double cos = this->cos_angle(grasp, point, vector, center);

		double cos = this->cos_angle(grasp, eigPoint, vector, center);
        if (cos > 0.001)
			grasp.VolDx += 1;
        else if (cos < -0.001)
			grasp.VolSx += 1;
    }

    if (grasp.VolDx <= obj->GetV0())
		grasp.VolDx = 0;

    if (grasp.VolSx <= obj->GetV0())
		grasp.VolSx = 0;

}

double GraspPlanner::cos_angle(Grasp& grasp, Eigen::Vector3d& point, Eigen::Vector3d& vector, Eigen::Vector3d& center){

	Eigen::Vector3d dir = point - center; 
    return (dir[0] * vector[0] + dir[1] * vector[1] + dir[2] * vector[2]) / (dir.norm() * vector.norm());

}

geometry_msgs::Pose GraspPlanner::EigenToRosPose(Eigen::Matrix4d poseMatrix){

    geometry_msgs::Pose rosPose;

	rosPose.position.x = poseMatrix(0, 3); 
	rosPose.position.y = poseMatrix(1, 3); 
	rosPose.position.z = poseMatrix(2, 3);

	Eigen::Quaternion<double> poseQuat = Eigen::Quaternion<double>(poseMatrix.block<3,3>(0,0));

	rosPose.orientation.w = poseQuat.w();
	rosPose.orientation.x = poseQuat.x();
	rosPose.orientation.y = poseQuat.y();
	rosPose.orientation.z = poseQuat.z();

    return rosPose;
}


void GraspPlanner::Spinner(){
	ROS_INFO("spinner");
	// Object* obj = new Object(this->nh);

}
