#include "../include/robot.hpp"

#include <math.h>
#include <iostream>
#include <list>

Link::Link(double alfa, double a, double d, double offset, double gamma, string path) {

	this->mAlfa = alfa;
	this->mA = a;
	this->mD = d;
	this->mOffset = offset;
	this->mGamma = gamma;
	if(path.length() > 0) this->mTriangularMesh = ParseSTLfile(path);
    else this->mTriangularMesh = vector<Triangle>();
    
}

Eigen::Matrix4d Link::DenavitHartenbergMatrix(double theta) {

	Eigen::Matrix4d output;
	double cosTheta = cos(theta + this->mOffset), sinTheta = sin(theta + this->mOffset);
	double cosAlfa = cos(this->mAlfa), sinAlfa = sin(this->mAlfa);
	output(0, 0) = cosTheta; output(0, 1) = -sinTheta*cosAlfa; output(0, 2) = sinTheta*sinAlfa;  output(0, 3) = this->mA*cosTheta;
	output(1, 0) = sinTheta; output(1, 1) = cosTheta*cosAlfa;  output(1, 2) = -cosTheta*sinAlfa; output(1, 3) = this->mA*sinTheta;
	output(2, 0) = .0;       output(2, 1) = sinAlfa;           output(2, 2) = cosAlfa;           output(2, 3) = this->mD;
	output(3, 0) = .0;       output(3, 1) = .0;                output(3, 2) = .0;                output(3, 3) = 1.;
	return output;

}

// RobotGripper::RobotGripper(bool isClosed, double alfa, double a, double d, double offset, double gamma, string path): Link(alfa,a,d,offset,gamma,path){
    
// // 	this->mPath = path;
// //	this->mRelativeDisplacement = pos;
//     this->mIsGripperClosed = false;
 
// }

Robot::Robot(ros::NodeHandle& input) {

	this->nh = input;

	// Definition of links

	Link link0(		  0.0,	 0.0, 	0.0,	0.0,	0.0, "src/safety_index/meshes/a0509_blue_stl/link0.stl");
	Link link1(		  -1.57, 0.0, 	0.1555, 0.0, 	1.0, "src/safety_index/meshes/a0509_blue_stl/link1.stl");
	Link link2(		  0.0,	 0.409,	0.0,	-1.57,	1.0, "src/safety_index/meshes/a0509_blue_stl/link2.stl");
	Link link3(		  1.57,	 0.0, 	0.0, 	1.57, 	1.5, "src/safety_index/meshes/a0509_blue_stl/link3.stl");
	Link link4(		  -1.57, 0.0,	0.367, 	0.0, 	1.5, "src/safety_index/meshes/a0509_blue_stl/link4.stl");
	Link link5(		  1.57,	 0.0,	0.0, 	0.0, 	1.5, "src/safety_index/meshes/a0509_blue_stl/link5.stl");
	Link link6(		  0.0,	 0.0, 	0.124, 	0.0, 	2.0, "src/safety_index/meshes/a0509_blue_stl/link6.stl");
	RobotGripper grip(0.0,	 0.0, 	0.0, 	-1.57, 	2.0, "src/safety_index/meshes/other/gripperr.stl");
	// mLinks.push_back(link0);
	this->mLinks.push_back(link1);
	this->mLinks.push_back(link2);
	this->mLinks.push_back(link3);
	this->mLinks.push_back(link4);
	this->mLinks.push_back(link5);
	this->mLinks.push_back(link6);
	this->mLinks.push_back(grip);

	vector<Triangle>::iterator it_t;
	list<Link>::iterator it_l;
	for (it_l = mLinks.begin(); it_l != mLinks.end(); ++it_l) {
		for (it_t = it_l->GetTriangularMesh()->begin(); it_t != it_l->GetTriangularMesh()->end(); ++it_t) {
			robot_mesh.push_back(*it_t);
		}
	}
	this->mBaseTriangularMesh = &robot_mesh;

	this->ComputeCoefficients(); // Ragaglia's coefficients: computed once because they are
								 // constant regardless the position of the triangle

	this->csf=0;
    this->mPosition = Eigen::Vector3d::Zero();
    this->mOrientation = Eigen::Matrix3d::Identity();
    this->mVelocity = Eigen::Vector3d::Zero();
    this->mAngularVelocity = Eigen::Vector3d::Zero();
    this->mIsInitialized = false;
	this->pBase = Eigen::Matrix4d::Identity(4,4);
	// this->pRobotName = new WCHAR[256]; //? WCHAR non viene trovato

	this->pointcloud = this->nh.advertise<sensor_msgs::PointCloud>("pc_topic",1);
	this->pointcloud_mesh = this->nh.advertise<sensor_msgs::PointCloud>("mesh_topic",1);
	this->CsfPublisher(); // initializing publishers
	this->sub_joint_state = this->nh.subscribe("/dsr01a0509/joint_states", 1, &Robot::ReadAndSetRobot, this);

	int ndof = this->mLinks.size();
	Eigen::VectorXd z(ndof); z.setZero();
	double zz[6] = {0,0,0,0,0,0};
	this->mJointPos = z;			// to initialize dimensions
	this->SetRobotState(zz);

}

void Robot::SetPosition(Eigen::Vector3d& pos, Eigen::Matrix3d& rot) {
    
    time_point<system_clock> st = system_clock::now();
    if(this->mIsInitialized) {
        Eigen::Matrix3d dotR, S;
        this->mVelocity = 1E3 * (pos - this->mPosition) / double(duration_cast<milliseconds>(st - this->mTimeStamp).count());
        dotR = 1E3 * (rot - this->mOrientation) / double(duration_cast<milliseconds>(st - this->mTimeStamp).count());
        S = dotR * rot.transpose();
        this->mAngularVelocity(0) = .5 * (S(2,1)-S(1,2));
        this->mAngularVelocity(1) = .5 * (S(0,2)-S(2,0));
        this->mAngularVelocity(2) = .5 * (S(1,0)-S(0,1));
    }
    this->mPosition = pos;
    this->mOrientation = rot;
    this->mTimeStamp = st;
    this->mIsInitialized = true;
    
}

void Robot::SetRobotState(double q[6]) {

	int k = 0, i = 0;
	JacobMatrix posJacob(3, this->nDOF()), oriJacob(3, this->nDOF());
	
    list<Link>::iterator it;
	// Eigen::Matrix4d A = *(this->pBase);
	Eigen::Matrix4d A = pBase;
	Eigen::Vector3d z, pk, pi, z0 = A.block<3, 1>(0, 2), p0 = A.block<3, 1>(0, 3);

	Eigen::Matrix4d Rot;
	for (it = this->mLinks.begin(); it != this->mLinks.end(); ++it) {
		this->mJointPos(k) = q[k];
		A = A * it->DenavitHartenbergMatrix(q[k]);
		it->SetCumulatedMatrix(A);
		k++;

		double vec[6] = {0,0,0,0,0,0};
		Rot = it->DenavitHartenbergMatrix(*vec);

	}
    
    // updating scene node properties
    Eigen::Vector3d pos = A.block<3,1>(0,3);
	Eigen::Matrix3d rot = A.block<3,3>(0,0);
    this->SetPosition(pos, rot);

	k = 0;
	for (it = this->mLinks.begin(); it != this->mLinks.end(); ++it) {
		posJacob.setZero();
		pk = it->GetCumulatedMatrix()->block<3, 1>(0, 3);
		for (i = 0; i <= k; i++) {
			if (i == 0) {
				z = z0;
				pi = p0;
			}
			else {
				z = this->GetCumulatedMatrix(i - 1)->block<3, 1>(0, 2);
				pi = this->GetCumulatedMatrix(i - 1)->block<3, 1>(0, 3);
			}
			posJacob.col(i) = z.cross(pk - pi);
			oriJacob.col(i) = z;
		}
		it->SetPosJacobian(posJacob);
		it->SetOriJacobian(oriJacob);
		k++;
	}


}

Link* Robot::GetLinkAt(unsigned int i) {
    
    list<Link>::iterator it;
    it = this->mLinks.begin();
    std::advance(it, i);
    return &(*it);
    
}

void Robot::UpdateRobotMesh(){

	Eigen::Matrix3d Rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Tran = Eigen::Vector3d::Zero(), a3, b3, c3, n3;
	vector<Triangle>::iterator it_t;
    Rot = this->GetBase().block <3, 3> (0, 0);
	Tran = this->GetBase().block <3, 1> (0, 3);
	*(this->mBaseTriangularMesh) = vector<Triangle>(); // To delete already existing mesh

	geometry_msgs::Point32 p1, p2, p3;
	vector<geometry_msgs::Point32> points;

	for(it_t = this->GetTriangularMesh()->begin(); it_t != this->GetTriangularMesh()->end(); ++it_t) {   // base mesh

		n3 = Rot * (*it_t->GetNormal());
		a3 = Rot * (*it_t->GetPoint1()) + Tran;
 		b3 = Rot * (*it_t->GetPoint2()) + Tran;
 		c3 = Rot * (*it_t->GetPoint3()) + Tran;

		this->mBaseTriangularMesh->push_back(Triangle(a3, b3, c3, n3));

	}
		
	for(unsigned int i = 0; i < this->nDOF(); ++i) {
			
		Rot = this->GetCumulatedMatrix(i)->block<3, 3>(0, 0);
		Tran = this->GetCumulatedMatrix(i)->block<3, 1>(0, 3);
		vector<Triangle> new_mesh; // Temporary mesh used to update the mesh of each link

		for(it_t = this->GetLinkAt(i)->GetTriangularMesh()->begin(); it_t != this->GetLinkAt(i)->GetTriangularMesh()->end(); ++it_t) {    // link mesh
		
			n3 = Rot * (*it_t->GetNormal());
			a3 = Rot * (*it_t->GetPoint1()) + Tran;
			b3 = Rot * (*it_t->GetPoint2()) + Tran;
			c3 = Rot * (*it_t->GetPoint3()) + Tran;
			this->mBaseTriangularMesh->push_back(Triangle(a3, b3, c3, n3)); // updating mesh of the whole robot

			new_mesh.push_back(Triangle(a3, b3, c3, n3)); // saving mesh of this link in new_mesh

			p1.x = a3(0); p1.y = a3(1); p1.z = a3(2); // these lines print the updated mesh on RVIZ
			p2.x = b3(0); p2.y = b3(1); p2.z = b3(2);
			p3.x = c3(0); p3.y = c3(1); p3.z = c3(2);
			points.push_back(p1);
			points.push_back(p2);
			points.push_back(p3);

        }

		this->GetLinkAt(i)->UpdateTriangularMesh(new_mesh); // updating the mesh of each link

    }

	this->mesh_points.points = points;  // these lines print the updated mesh on RVIZ
	std_msgs::Header h;
	h.frame_id = "world";
	this->mesh_points.header = h;

}

// To compute (sort of) Ragaglia's coefficients, this function is called only once
void Robot::ComputeCoefficients(){
	vector<Triangle>::iterator it_t;
	Eigen::Vector3d * point_A, * point_B, * point_C;
	Eigen::Vector3d point_P;
	Eigen::Vector3d dir_AB, seg_AB, seg_AC;

	unsigned int i;

	for(i = 0; i < this->nDOF(); ++i) {
		vector<double> coeff_0, coeff_x, coeff_y;
		vector<Eigen::Vector3d> temp_vec;

		for(it_t = this->GetLinkAt(i)->GetTriangularMesh()->begin(); it_t != this->GetLinkAt(i)->GetTriangularMesh()->end(); ++it_t) {

			point_A = it_t->GetPoint1();
			point_B = it_t->GetPoint2();
			point_C = it_t->GetPoint3();

			seg_AB = *point_B - *point_A;
			dir_AB = seg_AB / seg_AB.norm();
			seg_AC = *point_C - *point_A;

			double proj_len = seg_AC.dot(dir_AB); // length of segment AP

			point_P = *point_A + dir_AB * proj_len; // actual projection (point P)
			temp_vec.push_back(point_P); // the point P is the origin of the ref. frame fixed to the triangle
			
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
		
		this->c_0.push_back(coeff_0);
		this->c_x.push_back(coeff_x);
		this->c_y.push_back(coeff_y);
		this->ref_origins.push_back(temp_vec);
		
	}

}

// Computing the CSF knowing positions of triangles (mesh) and of the hand (pointcloud)
void Robot::ComputeSafetyInd(){
	vector<Eigen::Vector3d> body_points; // These points will be substituted by actual body points
	Eigen::Vector3d bp1;
	Eigen::Vector3d bp2;
	Eigen::Vector3d bp3;
	bp1 << -0.5, -0.5, 0.6;
	bp2 << -0.55, -0.5, 0.5;
	bp3 << -0.45, -0.55, 0.55;
	body_points.push_back(bp1);
	body_points.push_back(bp2);
	body_points.push_back(bp3);
	geometry_msgs::Point32 p1, p2, p3;
	p1.x = bp1(0); p1.y = bp1(1); p1.z = bp1(2);
	p2.x = bp2(0); p2.y = bp2(1); p2.z = bp2(2);
	p3.x = bp3(0); p3.y = bp3(1); p3.z = bp3(2);
	vector<geometry_msgs::Point32> points;
	points.push_back(p1); points.push_back(p2); points.push_back(p3);
	this->pc.points = points;
	std_msgs::Header h;
	h.frame_id = "world";
	this->pc.header = h;


	vector<Triangle>::iterator it_t;
	list<vector<Eigen::Vector3d>>::iterator it_origins = this->ref_origins.begin();
	list<vector<double>>::iterator it_c0 = this->c_0.begin(), it_cx = this->c_x.begin(), it_cy = this->c_y.begin();
	Eigen::Vector3d * point_A, * point_B, *point_C;
	unsigned int i, j, k;

	int n_points = body_points.size();

	Eigen::Vector3d dist_point; // distance of body point from origin of the triangle reference
	double dist_point_n;

	// Axes of the reference frames placed on each triangle
	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis;

	double dist_x; // distances of a body point along the axes
	double dist_y;
	double dist_z;

	this->csf=0; // cumulative safety field

	for(i = 0; i < this->nDOF(); ++i) {
		vector<double> coeff_0, coeff_x, coeff_y;
		vector<Eigen::Vector3d> temp_vec;
		k=0;

		for(it_t = this->GetLinkAt(i)->GetUpdatedTriangularMesh()->begin(); 
			it_t != this->GetLinkAt(i)->GetUpdatedTriangularMesh()->end(); ++it_t) {

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
					dist_point = body_points[j] - ref_origin;
					dist_point_n = dist_point.norm();

					dist_x = dist_point.dot(x_axis);
					dist_y = dist_point.dot(y_axis);
					dist_z = dist_point.dot(z_axis);
	
					this->csf = this->csf + this->GetLinkAt(i)->GetGamma() * 1e-4 *
											( it_c0->at(k) + it_cx->at(k) * dist_x + it_cy->at(k) * dist_y + 
											dist_point_n * dist_point_n);

				}

			}
			k++;

		}

		it_origins++; it_c0++; it_cx++; it_cy++;

	}
	msg_csf.data = this->csf;

}

void Robot::CsfPublisher()
{
    this->pub_csf = this->nh.advertise<std_msgs::Float64>("/csf_topic", 1);
	this->msg_csf.data = this->csf;
}

// Callback of subscriber of the topic of joint states, calling SetRobotState
// to update the robot mesh every time joints positions are changed
void Robot::ReadAndSetRobot(const sensor_msgs::JointState::ConstPtr& msg){
	double q[6];
	for(int i=0; i<6; i++){
		q[i] = msg->position[i];
	}
	this->SetRobotState(q);
}

void Robot::Spinner(){
	this->UpdateRobotMesh();
	this->pointcloud_mesh.publish(this->mesh_points);
	this->ComputeSafetyInd();
	this->pub_csf.publish(msg_csf);
	this->pointcloud.publish(this->pc);
}
