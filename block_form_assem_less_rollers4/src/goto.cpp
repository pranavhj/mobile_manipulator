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
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std;


nav_msgs::Odometry Odom_;
ros::Publisher cmd_vel_publisher_;
ros::Publisher odom_publisher_;
geometry_msgs::Pose odom_;
geometry_msgs::Pose2D position_;
float goal_x_=0;
float goal_y_=0;
float goal_theta_=0;
float kp_x_=0.6,ki_x_=0,kd_x_=0,prev_error_x_=0,total_error_x_=0;
float kp_theta_=0.0020,ki_theta_=0.00000,kd_theta_=0.002,prev_error_theta_=0,total_error_theta_=0;
float kp_y_=0.6,ki_y_=0,kd_y_=0,prev_error_y_=0,total_error_y_=0;


float PIDTheta(vector<float> euler_angles){
    float error_theta=goal_theta_-euler_angles[2];
    // if (abs(error_theta)>300){
    //     error_theta=(error_theta/abs(error_theta))*(360-abs(error_theta));
    // }

    float raw_pid_theta=kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_);
    raw_pid_theta=-raw_pid_theta;
    if (raw_pid_theta>0){
        raw_pid_theta=min(float(0.5),raw_pid_theta);

    }
    else{
        raw_pid_theta=max(float(-0.5),raw_pid_theta);
    }
    if(abs(error_theta)>300){
        raw_pid_theta=-raw_pid_theta/2.5;
        
    }
    total_error_theta_=total_error_theta_+error_theta;
    prev_error_theta_=error_theta;
    return raw_pid_theta;
}


float PIDX(float goal_x){
    float error_x=goal_x;//-odom_.position.x;

    float raw_pid_x=kp_x_*error_x+ki_x_*error_x+kd_x_*(error_x-prev_error_x_);
    raw_pid_x=raw_pid_x;
    if (raw_pid_x>0){
        raw_pid_x=min(float(0.5),raw_pid_x);

    }
    else{
        raw_pid_x=max(float(-0.5),raw_pid_x);
    }
    
    return raw_pid_x;
}


float PIDY(float goal_y){
    float error_y=goal_y;//-odom_.position.y;

    float raw_pid_y=kp_y_*error_y+ki_y_*error_y+kd_y_*(error_y-prev_error_y_);
    raw_pid_y=raw_pid_y;
    if (raw_pid_y>0){
        raw_pid_y=min(float(0.5),raw_pid_y);

    }
    else{
        raw_pid_y=max(float(-0.5),raw_pid_y);
    }
    
    return raw_pid_y;
}


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
    temp.push_back(180+(yaw*180/3.14159)-19.5);
    return temp;
}




void OdomCallback(const gazebo_msgs::ModelStates::ConstPtr& odom){
 
 odom_=odom->pose[1];

}


void PositionCallback(const geometry_msgs::Pose2D::ConstPtr& pos){
 
 position_=*pos;
 vector <float> temp;
 //temp=ConvertTo
 goal_x_=position_.x;
 goal_y_=position_.y;
 goal_theta_=position_.theta;

}


vector <float> ConvertPosition(vector <float> euler_angles){
    vector <float> position_wrt_robot;
    float theta=euler_angles[2]*3.142/180;
    float err_x=goal_x_-odom_.position.x;
    float err_y=goal_y_- odom_.position.y;
    position_wrt_robot.push_back((cos(theta)*err_x)+(sin(theta)*err_y));
    position_wrt_robot.push_back((-sin(theta)*err_x)+(cos(theta)*err_y));
    return position_wrt_robot;


}


void GetOdom_(){
    tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_.position.x;
    odom_trans.transform.translation.y = odom_.position.y;
    odom_trans.transform.translation.z = odom_.position.z;
    odom_trans.transform.rotation = odom_.orientation;
   
       //send the transform
    odom_broadcaster.sendTransform(odom_trans);



    Odom_.header.frame_id="odom";
    Odom_.pose.pose=odom_;
    Odom_.header.stamp=ros::Time::now();
    Odom_.pose.pose=odom_;
    //Odom_.child_frame_id = "base_link";
}

int main(int argc, char **argv)
{   
    
    ros::init(argc,argv,"Goto");

    // tf2_ros::TransformBroadcaster tfb;
    // geometry_msgs::TransformStamped transformStamped;
   
     
    // transformStamped.header.frame_id = "world11";
    // transformStamped.child_frame_id = "base_link";
    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    ros::NodeHandle n_;

    ros::Subscriber odom_subscriber_ = n_.subscribe("/gazebo/model_states", 10, OdomCallback);

    ros::Subscriber position_subscriber_ = n_.subscribe("/block_form_assem_less_rollers4/position", 10, PositionCallback);
    
    
    cmd_vel_publisher_=n_.advertise<geometry_msgs::Twist>("/block_form_assem_less_rollers4/cmd_vel",10);
    odom_publisher_=n_.advertise<nav_msgs::Odometry>("/block_form_assem_less_rollers4/odom",10);
    
    
    geometry_msgs::Twist cmd_vel_msg;
    while(n_.ok()){
        //cmd_vel_msg.linear.y=-0.7;
        //cmd_vel_msg.angular.z=0.5;
        //ros::spinOnce();
         
        vector <float> euler_angles=QuaterniontoEuler();  

        vector <float> position_wrt_robot=ConvertPosition(euler_angles);

        float raw_pid_theta;
        float raw_pid_x;
        float raw_pid_y;
        if(abs(goal_theta_-euler_angles[2])<10){
            raw_pid_theta=PIDTheta(euler_angles);        
            cmd_vel_msg.angular.z=raw_pid_theta;

            raw_pid_x=PIDX(position_wrt_robot[0]);
            cmd_vel_msg.linear.y=raw_pid_x;

            raw_pid_y=PIDY(position_wrt_robot[1]);
            cmd_vel_msg.linear.x=raw_pid_y;
        }
        else{
            raw_pid_theta=PIDTheta(euler_angles);        
            cmd_vel_msg.angular.z=raw_pid_theta;

            cmd_vel_msg.linear.y=0;
            cmd_vel_msg.linear.x=0;

        }

        

        


        //ROS_INFO_STREAM("error"<<goal_theta_-euler_angles[2],raw_pid_theta);//rpy
        //cout<<"error "<<goal_theta_-euler_angles[2]<<"raw_pid_theta "<<raw_pid_theta<<endl;
        cout<<"err x "<<goal_x_-odom_.position.x<<"  x  "<<raw_pid_x<<"  err y "<<goal_y_-odom_.position.y<<"  y  "<<raw_pid_y<<"  errtheta  "<<goal_theta_-euler_angles[2]<<"  theta "<<raw_pid_theta<<endl;
        //cout<<position_wrt_robot[0]<<"       "<<position_wrt_robot[1]<<"   "<<goal_theta_-euler_angles[2]<<endl;

        cmd_vel_publisher_.publish(cmd_vel_msg);
        GetOdom_();
        odom_publisher_.publish(Odom_);
        



        ros::spinOnce();
        //ROS_INFO_STREAM("y");
    }
   

   

   


}

