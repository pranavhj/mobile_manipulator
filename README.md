# mobile_manipulator
using ROS Kinetic and ubuntu 16.04
1. First unzip the packages block_form_assem_less_rollers4, demo_moveit_config, robot_control and place it in your workspace

2. Ensure that you have the universal robot package and place it in your workspace 

3. Run catkin_make

4. For launching the main simulation run 
roslaunch block_form_assem_less_rollers4 gazebo.launch

5. You should see a rover with a ur5 on top.

6. The errors you see on the terminal are majorly because of the version problem of gazebo so they can be ignored

7. You can check rostopic list and see all the topics where you can see 
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/state
which tells us the controllers for the arm have been defined.

If we try to move the arm, 
using,
roslaunch demo_moveit_config move_group.launch
rosrun robot_control robot_control_node 
in this simulation it does not work


But we can launch (close all other terminals)
roslaunch demo_moveit_config arm_controller_gazebo.launch 
which launches only the arm in the simulation
Now,


8. We can run (The demo_moveit_config is a moveit package made by me)
roslaunch demo_moveit_config move_group.launch
this launches the move_group interface

9. You can do a 
rosrun robot_control robot_control_node 
to see the robot moving which shows the controllers run well

10. The problem is that when we do the same with the combination file as above, the arm seems to start in a collision position and cannot be controlled further.

11. The urdf of the robot is present in block_form_assem_less_rollers4/urdf/block_form_assem_less_rollers4.urdf
The launch file for the whole robot is in block_form_assem_less_rollers4/launch/gazebo.launch
