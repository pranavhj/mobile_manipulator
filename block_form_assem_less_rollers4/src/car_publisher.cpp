#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
using namespace std;

float target1=0,target2=0,target3=0,target4=0;
control_msgs::JointControllerState wheel1_state;
control_msgs::JointControllerState wheel2_state;
control_msgs::JointControllerState wheel3_state;

int velocity_callback_flag=0;
float targets[3];
float filter_gain=1;
ros::Publisher wheel1_pub;
ros::Publisher wheel2_pub;
ros::Publisher wheel3_pub;
geometry_msgs::Twist command_vel;
float* matrixCalculation(float x, float y, float w);


void wheel1StateCallback(const control_msgs::JointControllerState::ConstPtr& odom){
 wheel1_state=*(odom);
}

void wheel2StateCallback(const control_msgs::JointControllerState::ConstPtr& odom){
 wheel2_state=*(odom);
}

void wheel3StateCallback(const control_msgs::JointControllerState::ConstPtr& odom){
 wheel3_state=*(odom);
}


void VelocityCallback(const geometry_msgs::Twist::ConstPtr& vel){
 command_vel=*(vel);
 velocity_callback_flag=1;
 //ROS_INFO_STREAM("Callback");
}


int main(int argc, char **argv)
{   ros::init(argc,argv,"wheel_pose_publisher_omni");
    std_msgs::Float64 f;
    ros::NodeHandle n;

    ros::Subscriber wheel1_subscriber = n.subscribe("/block_form_assem_less_rollers4/wheel1_joint_positiocontroller/state", 1000, &wheel1StateCallback);
    ros::Subscriber wheel2_subscriber = n.subscribe("/block_form_assem_less_rollers4/wheel2_joint_positiocontroller/state", 1000, &wheel2StateCallback);
    ros::Subscriber wheel3_subscriber = n.subscribe("/block_form_assem_less_rollers4/wheel3_joint_positiocontroller/state", 1000, &wheel3StateCallback);
    ros::Subscriber velocity_subscriber = n.subscribe("/block_form_assem_less_rollers4/cmd_vel", 1000, &VelocityCallback);
    
    
    ros::Rate rate(10);
    wheel1_pub=n.advertise<std_msgs::Float64>("/block_form_assem_less_rollers4/wheel1_joint_position_controller/command",10);
    wheel2_pub=n.advertise<std_msgs::Float64>("/block_form_assem_less_rollers4/wheel2_joint_position_controller/command",10);
    wheel3_pub=n.advertise<std_msgs::Float64>("/block_form_assem_less_rollers4/wheel3_joint_position_controller/command",10);
    
    std_msgs::Float64 wheel1_rad,wheel2_rad,wheel3_rad;
    wheel1_rad.data=0;
    wheel2_rad.data=0;
    wheel3_rad.data=0;

    std_msgs::Float64 fr=f,f0;
    fr.data=-0; 
    f.data=00.0;
    //f0=f;
    f0.data=0;
    float old_v1=0,old_v2=0,old_v3=0;

    while(n.ok()){

        float* velocity=matrixCalculation(command_vel.linear.x,command_vel.linear.y,command_vel.angular.z*0.1);
        float v1=*(velocity);
        float v2=*(velocity+1);
        float v3=*(velocity+2);
        if (velocity_callback_flag==0){
            v1=0;
            v2=0;
            v3=0;
        }
                
        velocity_callback_flag=0;
        
        v1=old_v1+filter_gain*(v1-old_v1);
        v2=old_v2+filter_gain*(v2-old_v2);
        v3=old_v3+filter_gain*(v3-old_v3);
        ROS_INFO_STREAM("velocity are"<<v1);
        ROS_INFO_STREAM(v2);
        ROS_INFO_STREAM(v3);

        if (abs(wheel1_state.error)<0.0001    &&   abs(wheel2_state.error)<0.0001   &&  abs(wheel3_state.error)<0.0001){
            //f.data+=0.05;
            //fr.data-=0.05;
            wheel1_rad.data-=v1;
            wheel2_rad.data+=v2;
            wheel3_rad.data+=v3;
        }

        wheel1_pub.publish(wheel1_rad);
        wheel2_pub.publish(wheel2_rad);
        wheel3_pub.publish(wheel3_rad);
        
        //ROS_INFO_STREAM("Publishing"<<f);
        ros::Duration(0.008).sleep();
        old_v1=v1;
        old_v2=v2;
        old_v3=v3;

        ros::spinOnce();
    }
   

   

   


}


float* matrixCalculation(float x, float y, float w)
{   //float targets[3];
    float body_twist;
    float body_velocity_x;
    float body_velocity_y;
    float wheel_radius;
    float wheel_distance; //chassis radius
    float motor_angVel_1; //angular velocity
    float motor_angVel_2;
    float motor_angVel_3;
    body_twist = w;
    body_velocity_x = x;
    body_velocity_y = y;
    wheel_radius = 10.16; /*in cms*/
    wheel_distance = 30; /*in cms*/
    motor_angVel_1 = ((-1*wheel_distance*body_twist) + body_velocity_x)/wheel_radius;
    motor_angVel_2 = ((-1*wheel_distance*body_twist) - (0.5*body_velocity_x) - (0.866*body_velocity_y))/wheel_radius;
    motor_angVel_3 = ((-1*wheel_distance*body_twist) - (0.5*body_velocity_x) + (0.866*body_velocity_y))/wheel_radius;
    targets[0]=(motor_angVel_1);
    targets[1]=(motor_angVel_2);
    targets[2]=(motor_angVel_3);
    return &targets[0];    
}