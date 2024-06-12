#ifndef group34_pkg2_COLLISION_OBEJCTS_H
#define group34_pkg2_COLLISION_OBEJCTS_H

#include <iostream>
#include <stdio.h>   
#include <cmath>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265

//initial pose of tiago
#define INITIAL_X -6.584825
#define INITIAL_Y 1.376042
#define INITIAL_Z -0.000740
#define INITIAL_ROLL 0.000234
#define INITIAL_PITCH 0.000284
#define INITIAL_YAW -0.000216

void addCollisionTables(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
void addCollisionObstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> coords);
void removeCollisionObstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

#endif //group34_pkg2_COLLISION_OBEJCTS_H
