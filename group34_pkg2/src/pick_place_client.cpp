#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <group34_pkg2/PickPlaceAction.h>
#include <string>
#include <vector>
#include <map>

typedef actionlib::SimpleActionClient<group34_pkg2::PickPlaceAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state, const group34_pkg2::PickPlaceResultConstPtr& result) {
    ROS_INFO("Finished with Result: [%s]", result->status.c_str());   
	ROS_INFO("State: [%s]", state.toString().c_str());
    ros::shutdown();
}

void activeCb() {
    ROS_INFO("Goal just went active");
}

void feedbackCb(const group34_pkg2::PickPlaceFeedbackConstPtr& feedback) {
    ROS_INFO("Feedback: %s", feedback->status.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_client");

    if (argc != 5) {
        ROS_ERROR("Usage: pick_place_client id task x y");
        return 1;
    }

    Client client("pick_place", true);
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    group34_pkg2::PickPlaceGoal goal;
	//ID	
	goal.id = atof(argv[1]);	
	//Task	
	goal.task = atof(argv[2]);
    // Position
    goal.x = atof(argv[3]);
    goal.y = atof(argv[4]);

	//goal.obstacles_coords = {0.78, -0.18, 0.76, -0.37, 3, 3, 4, 4};
	goal.obstacles_coords = {1, 1, 2, 2, 3, 3, 4, 4};
	
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}
