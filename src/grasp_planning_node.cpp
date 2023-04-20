// #include "../include/grasp_planning.hpp"
// #include "../include/robot.hpp"
#include "../include/Smartkit.hpp"

int main(int argc, char** argv){
    
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // GraspPlanner obj = GraspPlanner(nh);
    // Robot obj = Robot(nh);
    Smartkit obj = Smartkit();
    
    // ros::Rate r(10);
    // while (ros::ok()){
    //     ros::spinOnce();


    //     obj.Spinner();
    //     r.sleep();
    // }
}