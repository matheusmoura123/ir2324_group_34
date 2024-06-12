cmake_minimum_required(VERSION 3.0.2)
project(group34_pkg2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  group34_pkg1
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  apriltag_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
#add_message_files(
#		DIRECTORY msg
#   FILES
#   obj_coord.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
DIRECTORY srv
   FILES
   Objs.srv
   Attach.srv
 )

## Generate actions in the 'action' folder
add_action_files(
	 DIRECTORY action
   FILES
   PickPlace.action
   ReadTags.action
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS 
   	actionlib 
	actionlib_msgs 
	geometry_msgs 
	roscpp 
	sensor_msgs
	message_runtime
	moveit_core
    moveit_ros_planning_interface
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable for the server
#node_A
add_executable(node_A src/node_A.cpp src/collision_objects.h)
add_dependencies(node_A group34_pkg2_generate_messages_cpp  group34_pkg1_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(node_A ${catkin_LIBRARIES})

#pick_place_server
add_executable(read_tags_server src/read_tags_server.cpp)
add_dependencies(read_tags_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(read_tags_server ${catkin_LIBRARIES})

#pick_place_server
add_executable(pick_place_server src/pick_place_server.cpp src/collision_objects.cpp src/collision_objects.h)
add_dependencies(pick_place_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pick_place_server ${catkin_LIBRARIES})

#pick_place_client
add_executable(pick_place_client src/pick_place_client.cpp)
add_dependencies(pick_place_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pick_place_client ${catkin_LIBRARIES})
