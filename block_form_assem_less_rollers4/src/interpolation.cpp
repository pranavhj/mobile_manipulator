#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include <algorithm>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Pose2D.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;



ros::Publisher final_point_publisher_;
geometry_msgs::Pose odom_;
geometry_msgs::Pose2D position_,final_point_;




vector<float> QuaterniontoEuler(){
    vector<float> temp;
    geometry_msgs::Quaternion orientation=odom_.orientation;
    tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);   
    //ROS_INFO_STREAM(q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    temp.push_back(180+(roll*180/3.14159));
    temp.push_back(180+(pitch*180/3.14159));
    temp.push_back(180+(yaw*180/3.14159));
    return temp;
}







void FinalPointCallback(const geometry_msgs::Pose2D::ConstPtr& pos){
 
 final_point_=*pos;


}


int main(int argc, char **argv)
{   
	
    ros::init(argc,argv,"final_point");
    
    ros::NodeHandle n_;

    
    ros::Subscriber position_subscriber_ = n_.subscribe("/block_form_assem_less_rollers4/final_point", 10, FinalPointCallback);
    
    
    final_point_publisher_=n_.advertise<geometry_msgs::Pose2D>("/block_form_assem_less_rollers4/position",10);
    
    
    geometry_msgs::Twist cmd_vel_msg;
    while(n_.ok()){

    	//Do interpolation over here

        final_point_publisher_.publish(final_point_);
  
        ros::spinOnce();

    }
   

   

   


}


