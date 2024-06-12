#include <ros/ros.h>
#include <group34_pkg2/Objs.h>
#include <group34_pkg2/PickPlaceAction.h>
#include <group34_pkg2/ReadTagsAction.h>
#include <group34_pkg1/NavigateAction.h>
#include <actionlib/client/simple_action_client.h>
#include "collision_objects.h"
#include <cstdlib>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdio.h>   
#include <cmath>
#include <math.h>
#include <string>
#include <vector>
#include <map>

// Create a client to pick objs
typedef actionlib::SimpleActionClient<group34_pkg2::PickPlaceAction> PickPlaceClient;
typedef actionlib::SimpleActionClient<group34_pkg2::ReadTagsAction> ReadTagsClient;
typedef actionlib::SimpleActionClient<group34_pkg1::NavigateAction> NavigateClient;

void doneCb(const actionlib::SimpleClientGoalState& state, const group34_pkg2::PickPlaceResultConstPtr& result) {
    ROS_INFO("Finished with Result: [%s]", result->status.c_str());   
	ROS_INFO("State: [%s]", state.toString().c_str());
}
void activeCb() {
    ROS_INFO("Goal just went active");
}
void feedbackCb(const group34_pkg2::PickPlaceFeedbackConstPtr& feedback) {
    ROS_INFO("Feedback: %s", feedback->status.c_str());
}

group34_pkg2::PickPlaceGoal pp_goal_const(const int id, const int task, const double x, const double y, const double z, std::vector<double> obstacles_coords) {
	group34_pkg2::PickPlaceGoal goal;
	//ID	
	goal.id = id;	
	//Task	
	goal.task = task;
    // Position
    goal.x = x;
    goal.y = y;
    goal.z = z;
	//Obstacles_coords
	goal.obstacles_coords = obstacles_coords;
	return goal;
}

group34_pkg1::NavigateGoal n_goal_const(const double x, const double y, const double yaw_in) {
	group34_pkg1::NavigateGoal goal;
	// Position
    goal.x = x - INITIAL_X;
    goal.y = y - INITIAL_Y;
	goal.z = INITIAL_Z;

    // Euler angles (yaw_in) in degrees
    double roll = INITIAL_ROLL;
    double pitch = INITIAL_PITCH;
    double yaw = yaw_in*PI/180 - INITIAL_YAW;

    // Set the Euler angles directly in the goal
    goal.a = roll;
    goal.b = pitch;
    goal.gamma = yaw;
	return goal;
}

group34_pkg2::ReadTagsGoal rt_goal_const(const int id) {
	group34_pkg2::ReadTagsGoal goal;
    goal.id = id;
	return goal;
}

void scan(const int id, std::vector<double>& objects_coords, std::vector<double>& obstacles_coords) {
	// ReadTags Server Client setup
	ReadTagsClient read_tags_client("read_tags", true);
    ROS_INFO("Waiting for ReadTags server to start.");
    read_tags_client.waitForServer();
    ROS_INFO("ReadTags server started.");

	//Initialize ReadTags Result
	group34_pkg2::ReadTagsResultConstPtr result; 

	read_tags_client.sendGoal(rt_goal_const(id));
	read_tags_client.waitForResult();
	result = read_tags_client.getResult();

	//objects_coords.clear();
	if (result->objects_coords.size() > 0) {
		for (size_t i = 0; i < result->objects_coords.size(); ++i) {
			objects_coords[i] = result->objects_coords[i];
    	}
		ROS_INFO("Objects positions: ");
    	for (int i = 0; i < objects_coords.size(); i=i+2) {
    		ROS_INFO("%d: x = %f, y = %f", (i/2)+1, objects_coords[i], objects_coords[i+1]);
		}
	}
	else ROS_INFO("No objects found");

	//obstacles_coords.clear();
	if (result->obstacles_coords.size() > 0) {
		for (size_t i = 0; i < result->obstacles_coords.size(); ++i) {
			obstacles_coords[i] = result->obstacles_coords[i];
    	}
		ROS_INFO("Obstacles positions: ");
    	for (int i = 0; i < obstacles_coords.size(); i=i+2) {
    		ROS_INFO("%d: x = %f, y = %f", (i/2)+1, obstacles_coords[i], obstacles_coords[i+1]);
		}
	}
	else ROS_INFO("No obstacles found");
	
	read_tags_client.cancelGoal();

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;

    // Service client setup
    ros::ServiceClient client = nh.serviceClient<group34_pkg2::Objs>("/human_objects_srv");
    group34_pkg2::Objs srv;

    // Set up the service request
    srv.request.ready = true;
    srv.request.all_objs = true; // Set this to true if you want to request all objects	
	
	// Pick&Place Server Client setup
	PickPlaceClient pick_place_client("pick_place", true);
    ROS_INFO("Waiting for Pick&Place server to start.");
    pick_place_client.waitForServer();
    ROS_INFO("Pick&Place server started.");

    // Navigate Server Client setup
	NavigateClient navigate_client("navigate", true);
    ROS_INFO("Waiting for Navigate server to start.");
    navigate_client.waitForServer();
    ROS_INFO("Navigate server started.");

    //Initialize obstacle coord
	std::vector<double> objects_coords = {0, 0, 0};
	std::vector<double> obstacles_coords = {11, 11, 12, 12, 13, 13, 14, 14};



    if (client.call(srv))
    {
		pick_place_client.sendGoal(pp_goal_const(1, 3, 0, 0, 0, obstacles_coords), &doneCb, &activeCb, &feedbackCb);
	    pick_place_client.waitForResult();
        ROS_INFO("Received object IDs:");
        for (const int32_t& id : srv.response.ids)
        {
            ROS_INFO("ID: %d", id);

            // Go to the initial pose 
            navigate_client.sendGoal(n_goal_const(2.0,  1.0, -90));
            navigate_client.waitForResult();
            // pick_place_client.sendGoal(pp_goal_const(id, task, x, y, obstacles_coords));
			//task = 1 -> PICK, 
			//task = 2 -> PLACE
			//task = 3 -> SCAN
            pick_place_client.sendGoal(pp_goal_const(1, 3, 0, 0, 0, obstacles_coords), &doneCb, &activeCb, &feedbackCb);
	        pick_place_client.waitForResult();

			double x, y, z;

            switch (id) {
				case 1:  // Blue
                    navigate_client.sendGoal(n_goal_const(1.5, -0.5, -85)); // Navigate TO PICK OBJECT BLU
     				navigate_client.waitForResult();
					scan(id, objects_coords, obstacles_coords); //SCANNING
					x = objects_coords[0];
					y = objects_coords[1];
					z = objects_coords[2];
					ros::Duration(1).sleep(); 
					ROS_INFO("x y z: %f %f %f", x, y, z);

    				pick_place_client.sendGoal(pp_goal_const(1, 1, x, y, z, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();
					navigate_client.sendGoal(n_goal_const(1.5, -0.4, -179));
     				navigate_client.waitForResult();
     				navigate_client.sendGoal(n_goal_const(2.0, 1.5, 45)); // Navigate to intermediate position
					navigate_client.waitForResult();
					navigate_client.sendGoal(n_goal_const(5.9, 1.8, -95)); // Navigate to RELEASE OBJECT BLU
					navigate_client.waitForResult();
					pick_place_client.sendGoal(pp_goal_const(1, 2, 0, 0, 0, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();

                    break;
                case 2:  // Green
                    navigate_client.sendGoal(n_goal_const(2.3, -2.5, -90)); // Navigate to INTERMEDIATE position
     				navigate_client.waitForResult();
                    navigate_client.sendGoal(n_goal_const(0.8, -2.7, 70)); // Navigate TO PICK OBJECT GREEN
     				navigate_client.waitForResult();
					scan(id, objects_coords, obstacles_coords); //SCANNING
					x = objects_coords[0];
					y = objects_coords[1];
					z = objects_coords[2];
					ros::Duration(1).sleep();
					ROS_INFO("x y z: %f %f %f", x, y, z);

    				pick_place_client.sendGoal(pp_goal_const(2, 1, x, y, z, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();
                    navigate_client.sendGoal(n_goal_const(2.3, -2.5, 0)); // Navigate to INTERMEDIATE position
     				navigate_client.waitForResult();
					pick_place_client.sendGoal(pp_goal_const(1, 3, 0, 0, 0, obstacles_coords), &doneCb, &activeCb, &feedbackCb); //TUCK ARM
	        		pick_place_client.waitForResult();
					navigate_client.sendGoal(n_goal_const(2.0, 1.5, 45)); // Navigate to INTERMEDIATE position
     				navigate_client.waitForResult();
                    navigate_client.sendGoal(n_goal_const(4.9, 1.8, -95)); // Navigate to RELEASE OBJECT GREEN
     				navigate_client.waitForResult(); 
					pick_place_client.sendGoal(pp_goal_const(2, 2, 0, 0, 0, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();

                    break;
                case 3:  // Red
                    navigate_client.sendGoal(n_goal_const(1, -0.6, -85)); // Navigate TO PICK OBJECT RED
     				navigate_client.waitForResult();
					scan(id, objects_coords, obstacles_coords); //SCANNING
					x = objects_coords[0];
					y = objects_coords[1];
					z = objects_coords[2];
					ros::Duration(1).sleep();
					ROS_INFO("x y z: %f %f %f", x, y, z);

    				pick_place_client.sendGoal(pp_goal_const(3, 1, x, y, z, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();
					navigate_client.sendGoal(n_goal_const(2.0, 1.5, 45)); // Navigate to INTERMEDIATE position
     				navigate_client.waitForResult();
                    navigate_client.sendGoal(n_goal_const(3.9, 1.8, -95)); // Navigate to RELEASE OBJECT RED
     				navigate_client.waitForResult(); 
					pick_place_client.sendGoal(pp_goal_const(3, 2, 0, 0, 0, obstacles_coords),  &doneCb, &activeCb, &feedbackCb);
					pick_place_client.waitForResult();

                    break;
                default:
                    ROS_ERROR("Invalid ID received: %d", id);
                    break;
            }
        }
		ROS_INFO("Task completed");
    }
    else
    {
        ROS_ERROR("Failed to call service /human_objects_srv");
        return 1;
    }

    return 0;
}
