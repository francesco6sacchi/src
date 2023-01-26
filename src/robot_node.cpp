#include "../include/robot.hpp"

int main(int argc, char** argv){

	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;

	Robot obj = Robot(nh);

	ros::Rate r(10);

	while(ros::ok())
	{
		ros::spinOnce();

		obj.Spinner();

		r.sleep();

	}
    
    return 0;
}
