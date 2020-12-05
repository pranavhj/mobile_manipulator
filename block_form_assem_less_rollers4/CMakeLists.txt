cmake_minimum_required(VERSION 2.8.3)

project(block_form_assem_less_rollers4)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/car_publisher.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )


add_executable(goto src/goto.cpp)

target_link_libraries(goto
   ${catkin_LIBRARIES}
 )


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
