

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include<vector>
#include <ros/ros.h>
#include <algorithm>
#include <ros/ros.h>


#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>


void Moveto(ros::NodeHandle node,geometry_msgs::Pose pose1);
int main(int argc, char** argv) {
    //--Work with MoveIt
    ros::init(argc, argv, "moveto");
    ros::NodeHandle node("/arm_controller/follow_joint_trajectory");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.77;
    pose.position.y = 0.10;
    pose.position.z = +0.1;
    ROS_INFO_STREAM("Moving to pose.......");
    for(int i=0;i<20;i++){

        Moveto(node,pose);
        pose.position.y-=0.01;
    }
        ros::Duration(4.0).sleep();
    
    //Moveto(node,pose);
    std::cout<<"Done"<<std::endl;

    //gripperToggle(true);

    ros::shutdown();

    return 0;
}
void Moveto(ros::NodeHandle node,geometry_msgs::Pose pose1)
{
static const std::string PLANNING_GROUP = "manipulator";
moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","robot_description",node);
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// We can print the name of the reference frame for this robot.
//ROS_INFO_NAMED("Lecture5", "Planning frame: %s", move_group.getPlanningFrame().c_str());
// We can also print the name of the end-effector link for this group.
//ROS_INFO_NAMED("Lecture5", "End effector link: %s", move_group.getEndEffectorLink().c_str());

// We can plan a motion for this group to a desired pose for the
// end-effector.
geometry_msgs::Pose target_pose1;
//target_pose1.orientation.w = 1.0;
//target_pose1.position.x = 0.200012;
//target_pose1.position.y = 0.483010;
//target_pose1.position.z = +0.624102;
target_pose1=pose1;
ROS_INFO_STREAM("Move to target_pose1");
move_group.setPoseTarget(target_pose1);


// Now, we call the planner to compute the plan and visualize it.
// Note that we are just planning, not asking move_group
// to actually move the robot.
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

move_group.move();
}
