#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/shared_ptr.hpp>
#include <ros/topic.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <group34_pkg2/PickPlaceAction.h>
#include <string>
#include <vector>
#include <cctype>
#include <map>
#include "collision_objects.h"
#include "group34_pkg2/Attach.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointControllerClient;

class PickPlaceServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<group34_pkg2::PickPlaceAction> as_;
    std::string action_name_;
    group34_pkg2::PickPlaceFeedback feedback_;
    group34_pkg2::PickPlaceResult result_;
	JointControllerClient gripper_controller_client_, torso_controller_client_, head_controller_client_, arm_controller_client_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::ServiceClient attach_client = nh_.serviceClient<group34_pkg2::Attach>("/link_attacher_node/attach");
	ros::ServiceClient detach_client = nh_.serviceClient<group34_pkg2::Attach>("/link_attacher_node/detach");
	group34_pkg2::Attach attacher_srv;

public:
    PickPlaceServer(std::string name)
        : as_(nh_, name, boost::bind(&PickPlaceServer::executeCB, this, _1), false),
          action_name_(name),
		  gripper_controller_client_("/gripper_controller/follow_joint_trajectory", true),
		  torso_controller_client_("/torso_controller/follow_joint_trajectory", true),
		  head_controller_client_("/head_controller/follow_joint_trajectory", true),
		  arm_controller_client_("/arm_controller/follow_joint_trajectory", true)
	{	
		//add tables collisions
		addCollisionTables(planning_scene_interface);	
        as_.start();
    }
	
	bool move_to(const geometry_msgs::PoseStamped goal_pose) {	
  		std::vector<std::string> torso_arm_joint_names;
  		//select group of joints
  		moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
 		//choose your preferred planner
  		group_arm_torso.setPlannerId("SBLkConfigDefault");
  		group_arm_torso.setPoseReferenceFrame("base_footprint");
  		group_arm_torso.setPoseTarget(goal_pose);
		
		feedback_.status = "Planning move";
        as_.publishFeedback(feedback_);

   		group_arm_torso.setStartStateToCurrentState();
  		group_arm_torso.setMaxVelocityScalingFactor(0.5);

  		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  		//set maximum time to find a plan
  		group_arm_torso.setPlanningTime(5.0);
  		bool success = bool(group_arm_torso.plan(my_plan));
  		if(!success) {
			feedback_.status = "No plan found";
       		as_.publishFeedback(feedback_);
			return false;
		}
		
		feedback_.status = "Plan found";
       	as_.publishFeedback(feedback_);

  		// Execute the plan
  		moveit::core::MoveItErrorCode e = group_arm_torso.move();
  		if(!bool(e)) {
			feedback_.status = "PickPlace failed";
       		as_.publishFeedback(feedback_);
			return false;
		}

		feedback_.status = "PickPlace succeeded";
       	as_.publishFeedback(feedback_);

		return true;
	}

	// Function to Generate goal
	control_msgs::FollowJointTrajectoryGoal goal(std::vector<std::string> joints, std::vector<double> value) {
		int size = joints.size();
		
		//Generate goal
		control_msgs::FollowJointTrajectoryGoal goal;
		
		goal.trajectory.header.frame_id = "base_footprint";
		// One waypoint in this goal trajectory		
		goal.trajectory.points.resize(1);
  		
		int index = 0;
		goal.trajectory.points[index].positions.resize(size);
		goal.trajectory.points[index].velocities.resize(size);
  		
  		for (int i = 0; i < size; ++i) {
			// The joint names, which apply to all waypoints
			goal.trajectory.joint_names.push_back(joints[i]);
			// First trajectory point
			goal.trajectory.points[index].positions[i] = value[i];
			// Velocities
    		goal.trajectory.points[index].velocities[i] = 0.05;
  		}
  		// To be reached 1 second after starting along the trajectory
  		goal.trajectory.points[index].time_from_start = ros::Duration(1.0);	

		return goal;
	}
	
	// Function to move to scan pose
	void move_to_scan_pose() {

		std::vector<double> torso_height = {1}, 
							head_angle = {0, -0.7},
							arm_values = {0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00};
		std::vector<std::string> torso = {"torso_lift_joint"}, 
								head = {"head_1_joint", "head_2_joint"},
								arm = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"};
			
		//Send Goal
		torso_controller_client_.sendGoal(goal(torso, torso_height));
		torso_controller_client_.waitForResult();
		arm_controller_client_.sendGoal(goal(arm, arm_values));
		arm_controller_client_.waitForResult();
		head_controller_client_.sendGoal(goal(head, head_angle));
		head_controller_client_.waitForResult();

		feedback_.status = "scan pose succeeded";
       	as_.publishFeedback(feedback_);
	}

	
	void pickPlaceRoutine(const int task, const std::vector<std::string> name, const std::vector<double> coords, const std::vector<geometry_msgs::Quaternion> orientation) {

		std::vector<double> torso_height = {1},
							head_angle = {0, 0};
		std::vector<std::string> torso = {"torso_lift_joint"}, 
								head = {"head_1_joint", "head_2_joint"};

		head_controller_client_.sendGoal(goal(head, head_angle));
		head_controller_client_.waitForResult();
				
		//coords = {pick_x, pick_y, safe_z, pick_z, pick_grip, trans_x, trans_y};
		//orientation = {grip_orientation_pickplace, grip_orientation_trans};
  		geometry_msgs::PoseStamped goal_pose;
		goal_pose.header.frame_id = "base_footprint";

		//Initial Safe Pose
		goal_pose.pose.position.x = coords[0];
		goal_pose.pose.position.y = coords[1];
		goal_pose.pose.position.z = coords[2];
		goal_pose.pose.orientation = orientation[0];
		bool safe = move_to(goal_pose);
		if (!safe){
			result_.status = "APPROACH FAILED";
			as_.setAborted(result_);
			return;
		}

		//Approach Pose
		
		goal_pose.pose.position.z = coords[3];
		bool approach = move_to(goal_pose);
		if (!approach){
			result_.status = "PICKING/PLACING FAILED";
			as_.setAborted(result_);
			return;
		}
		

			
		//GRAB & RELEASE
		// Generates and send the goal for the TIAGo's gripper
		std::vector<double> gripper_size = {coords[4], coords[4]} ;
		std::vector<std::string> gripper = {"gripper_left_finger_joint", "gripper_right_finger_joint"};	
		gripper_controller_client_.sendGoal(goal(gripper, gripper_size));
		gripper_controller_client_.waitForResult();

		//ATTACH & DETACH
		attacher_srv.request.model_name_1 = "tiago";
		attacher_srv.request.link_name_1  = "arm_7_link";
		attacher_srv.request.model_name_2 = name[0];
		attacher_srv.request.link_name_2  = name[1];
		if (task == 1) {
			attach_client.call(attacher_srv);
			attach_client.waitForExistence();
		} else if (task == 2) {
			detach_client.call(attacher_srv);
			detach_client.waitForExistence();
		}
		
		//Transport Pose
		//move_to_scan_pose();

   		goal_pose.pose.position.x = coords[0];
		goal_pose.pose.position.y = coords[1];
		goal_pose.pose.position.z = coords[2];
		goal_pose.pose.orientation = orientation[1];
		bool transport = move_to(goal_pose);
		if (!transport){
			result_.status = "TRANSPORT FAILED";
			as_.setAborted(result_);
			return;
		}

		//move_to_scan_pose();

	}

		
    void executeCB(const group34_pkg2::PickPlaceGoalConstPtr &goal) {

		gripper_controller_client_.waitForServer();
		torso_controller_client_.waitForServer();
		head_controller_client_.waitForServer();
		arm_controller_client_.waitForServer();
		
		geometry_msgs::PoseStamped goal_pose;
		goal_pose.header.frame_id = "base_footprint";

		double pick_grip,
		pick_x     =  goal->x - 0.2,
		pick_y     =  goal->y,
		pick_z     =  goal->z,
		safe_z     = 1.2,
		trans_x    = 0.4,
		trans_y    = 0.1,	
		place_x    = 0.6,
		place_y    = 0.0,
		place_z    = 1,
		place_grip = 0.05;

		//Horizontal sideways grip
        geometry_msgs::Quaternion grip_orientation_pickplace = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0),
		//Vertical sideways grip
        grip_orientation_trans = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
		
		std::string obj_model_name, obj_link_name;		
		
		switch (goal->id) {
        	case 1:  //Blue	
				pick_grip   =  0.03;
				obj_model_name = "Hexagon";
				obj_link_name = "Hexagon_link";
                break;
        	case 2:  //Green
				pick_z = pick_z + 0.05;
				pick_grip   =  0.03;
				obj_model_name = "Triangle";
				obj_link_name = "Triangle_link";
				break;
        	case 3:  //Red
				pick_grip   =  0.03;
				obj_model_name = "cube";
				obj_link_name = "cube_link";
				break;
			default:
				feedback_.status = "Wrong input for id variable";
       			as_.publishFeedback(feedback_);
				result_.status = "FAILED";
				as_.setAborted(result_);
 				break;
		}
			
		std::vector<double> coords(7);
		std::vector<geometry_msgs::Quaternion> orientation(2);
		std::vector<std::string> name = {obj_model_name, obj_link_name};
		switch (goal->task) {
        	case 1:  //Pick
				//add obstacles collisions
				addCollisionObstacles(planning_scene_interface, goal->obstacles_coords);
				coords = {pick_x, pick_y, safe_z, pick_z, pick_grip, trans_x, trans_y};
				orientation = {grip_orientation_pickplace, grip_orientation_trans};
  				pickPlaceRoutine(goal->task, name, coords, orientation);
				//remove obstacles collisions
				removeCollisionObstacles(planning_scene_interface);
                break;
        	case 2:  //Place
				coords = {place_x, place_y, safe_z, place_z, place_grip, trans_x, trans_y};
				orientation = {grip_orientation_pickplace, grip_orientation_trans};
  				pickPlaceRoutine(goal->task, name, coords, orientation);
				break;
        	case 3:  //Scan
				move_to_scan_pose();
				break;
			default:
				feedback_.status = "Wrong input for task variable";
       			as_.publishFeedback(feedback_);
				result_.status = "FAILED";
				as_.setAborted(result_);
 				break;
		}	
		
		result_.status = "SUCCEEDED";
		as_.setSucceeded(result_);	
    }
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_server");
    PickPlaceServer server("pick_place");
    ros::spin();
    return 0;
}
