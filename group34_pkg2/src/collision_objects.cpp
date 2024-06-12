#include "collision_objects.h"

void addCollisionTables(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {

  // Create vector to hold all collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Define pick_table
  // Add the object
  collision_objects[0].id = "pick_table";
  collision_objects[0].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.92;
  collision_objects[0].primitives[0].dimensions[1] = 0.92;
  collision_objects[0].primitives[0].dimensions[2] = 0.8;
  // Define the pose of the table.
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x =  1.245143 - INITIAL_X;
  collision_objects[0].primitive_poses[0].position.y = -1.613171 - INITIAL_Y;
  collision_objects[0].primitive_poses[0].position.z =  0.375;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[0].operation = collision_objects[0].ADD;

  // Define place_table_r
  // Add the object
  collision_objects[1].id = "place_table_r";
  collision_objects[1].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(2);
  collision_objects[1].primitives[0].dimensions[0] = 0.70;
  collision_objects[1].primitives[0].dimensions[1] = 0.215;
  // Define the pose.
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x =  4.007962 - INITIAL_X;
  collision_objects[1].primitive_poses[0].position.y =  1.015966 - INITIAL_Y;
  collision_objects[1].primitive_poses[0].position.z =  0.345000;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[1].operation = collision_objects[1].ADD;

  // Define place_table_g
  // Add the object
  collision_objects[2].id = "place_table_g";
  collision_objects[2].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[2].primitives[0].dimensions.resize(2);
  collision_objects[2].primitives[0].dimensions[0] = 0.70;
  collision_objects[2].primitives[0].dimensions[1] = 0.215;
  // Define the pose.
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x =  5.007404 - INITIAL_X;
  collision_objects[2].primitive_poses[0].position.y =  1.015966 - INITIAL_Y;
  collision_objects[2].primitive_poses[0].position.z =  0.345000;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[2].operation = collision_objects[2].ADD;

  // Define place_table_b
  // Add the object
  collision_objects[3].id = "place_table_b";
  collision_objects[3].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.70;
  collision_objects[3].primitives[0].dimensions[1] = 0.215;
  // Define the pose.
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x =  6.007146 - INITIAL_X;
  collision_objects[3].primitive_poses[0].position.y =  1.015966 - INITIAL_Y;
  collision_objects[3].primitive_poses[0].position.z =  0.345000;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[3].operation = collision_objects[3].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}


void addCollisionObstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<double> coords) {

  // Create vector to hold all collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Define Obstacle1
  // Add the object
  collision_objects[0].id = "Obstacle0";
  collision_objects[0].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = 0.20;
  collision_objects[0].primitives[0].dimensions[1] = 0.04;
  // Define the pose.
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = coords[0];
  collision_objects[0].primitive_poses[0].position.y = coords[1];
  collision_objects[0].primitive_poses[0].position.z = 0.901823;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[0].operation = collision_objects[0].ADD;

  // Define Obstacle2
  // Add the object
  collision_objects[1].id = "Obstacle1";
  collision_objects[1].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(2);
  collision_objects[1].primitives[0].dimensions[0] = 0.20;
  collision_objects[1].primitives[0].dimensions[1] = 0.04;
  // Define the pose. 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = coords[2];
  collision_objects[1].primitive_poses[0].position.y = coords[3];
  collision_objects[1].primitive_poses[0].position.z =  0.901823;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[1].operation = collision_objects[1].ADD;

  // Define Obstacle3
  // Add the object
  collision_objects[2].id = "Obstacle2";
  collision_objects[2].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions.
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[2].primitives[0].dimensions.resize(2);
  collision_objects[2].primitives[0].dimensions[0] = 0.20;
  collision_objects[2].primitives[0].dimensions[1] = 0.04;
  // Define the pose. 
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = coords[4];
  collision_objects[2].primitive_poses[0].position.y = coords[5];
  collision_objects[2].primitive_poses[0].position.z =  0.901823;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[2].operation = collision_objects[2].ADD;

  // Define Obstacle4
  // Add the object
  collision_objects[3].id = "Obstacle3";
  collision_objects[3].header.frame_id = "base_footprint";
  // Define the primitive and its dimensions. 
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.20;
  collision_objects[3].primitives[0].dimensions[1] = 0.04;
  // Define the pose. 
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = coords[6];
  collision_objects[3].primitive_poses[0].position.y = coords[7];
  collision_objects[3].primitive_poses[0].position.z =  0.901823;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  // Add the object
  collision_objects[3].operation = collision_objects[3].ADD;
	
  planning_scene_interface.addCollisionObjects(collision_objects);
}

void removeCollisionObstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
	std::vector< std::string > obstacles = {"Obstacle0", "Obstacle1", "Obstacle2", "Obstacle3"};
	planning_scene_interface.removeCollisionObjects(obstacles);
}





