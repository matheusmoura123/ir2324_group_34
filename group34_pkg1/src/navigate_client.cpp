#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <group34_pkg1/NavigateAction.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include "cilinder_finder.h"


typedef actionlib::SimpleActionClient<group34_pkg1::NavigateAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state, const group34_pkg1::NavigateResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    vector<double> obstacle_positions;
	if (result->obstacles_positions.size() > 0) {
		for (size_t i = 0; i < result->obstacles_positions.size(); ++i) {
        	obstacle_positions.push_back(result->obstacles_positions[i]);
    	}
		ROS_INFO("Obstacle positions: ");
    	for (int i = 0; i < obstacle_positions.size(); i=i+2) {
    		ROS_INFO("%d: x = %f, y = %f", (i/2)+1, obstacle_positions[i], obstacle_positions[i+1]);
		}
	}
	else ROS_INFO("No movable obstacle found");
    ros::shutdown();
}

void activeCb() {
    ROS_INFO("Goal just went active");
}

void feedbackCb(const group34_pkg1::NavigateFeedbackConstPtr& feedback) {
    ROS_INFO("Feedback: %s", feedback->status.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_client");

    if (argc != 4) {
        ROS_ERROR("Usage: navigate_client x y yaw(in degrees)");
        return 1;
    }

    Client client("navigate", true);
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    group34_pkg1::NavigateGoal goal;

    // Position
    goal.x = atof(argv[1]) - INITIAL_X;
    goal.y = atof(argv[2]) - INITIAL_Y;
	goal.z = INITIAL_Z;

    // Euler angles (yaw) in degrees
    double roll = INITIAL_ROLL;
    double pitch = INITIAL_PITCH;
    double yaw = atof(argv[3])*PI/180 - INITIAL_YAW;

    // Set the Euler angles directly in the goal
    goal.a = roll;
    goal.b = pitch;
    goal.gamma = yaw;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}
