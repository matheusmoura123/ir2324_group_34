#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <group34_pkg1/NavigateAction.h>
#include <sensor_msgs/LaserScan.h>
#include "cilinder_finder.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigateServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<group34_pkg1::NavigateAction> as_;
    std::string action_name_;
    group34_pkg1::NavigateFeedback feedback_;
    group34_pkg1::NavigateResult result_;
    MoveBaseClient move_base_client_;
	ros::Subscriber sub_;
	vector<vector<double>> cilinders_wf;
	vector<double> tiago_pose_world; //{x, y, yaw(rads)}
	vector<double> ranges;

public:
    NavigateServer(std::string name)
        : as_(nh_, name, boost::bind(&NavigateServer::executeCB, this, _1), false),
          action_name_(name),
          move_base_client_("move_base", true)
    {	
		//subscribe to the data topic of interest
 		sub_ = nh_.subscribe("/scan_raw", 1, &NavigateServer::getCB, this);
        as_.start();
    }

	void getCB(const sensor_msgs::LaserScanConstPtr& scan){
		ranges.clear();
		ranges.insert(ranges.begin(), scan->ranges.begin(), scan->ranges.end());
	}

	void scanningRoutine(const group34_pkg1::NavigateGoalConstPtr &goal) {
		tiago_pose_world.clear();			
	    tiago_pose_world.push_back(goal->x + INITIAL_X);
	    tiago_pose_world.push_back(goal->y + INITIAL_Y);
	    tiago_pose_world.push_back(goal->gamma);
	    // make sure that the action hasn't been canceled
		if (!as_.isActive()) return;
	    bool scan_success = true;		
  		cilinders_wf = cilinder_scan(ranges, tiago_pose_world, LASER_DELTA, CILINDER_R, LASER_ANGLE_MIN);
        result_.obstacles_positions.clear();
	    if (cilinders_wf.size() > 1) {
		    for (size_t i = 0; i < cilinders_wf.size(); i++) {
		    	if (as_.isPreemptRequested() || !ros::ok()) {
      		    		ROS_INFO("%s: Preempted", action_name_.c_str());
      		    		as_.setPreempted();
      		    		scan_success = false;
      		    		break;
    			}
  			    result_.obstacles_positions.push_back(cilinders_wf[i][0]);
		    	result_.obstacles_positions.push_back(cilinders_wf[i][1]);
		    }
	    }

  		if(scan_success){
            feedback_.status = "Scan: Succeeded";
  		} else {
     		feedback_.status = "Scan: Failed";
        }
        as_.publishFeedback(feedback_);
        as_.setSucceeded(result_);
    } 

    void executeCB(const group34_pkg1::NavigateGoalConstPtr &goal) {
        move_base_client_.waitForServer();

        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.frame_id = "map";
        move_base_goal.target_pose.header.stamp = ros::Time::now();

        move_base_goal.target_pose.pose.position.x = goal->x;
        move_base_goal.target_pose.pose.position.y = goal->y;
        move_base_goal.target_pose.pose.position.z = goal->z;
		 		
        tf2::Quaternion quaternion;
        quaternion.setRPY(goal->a, goal->b, goal->gamma);  // Convert Euler angles to Quaternion

        move_base_goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

        // Send goal and feedback that the robot is moving to the target
        feedback_.status = "The robot is moving to the target pose.";
        as_.publishFeedback(feedback_);
        
        move_base_client_.sendGoal(move_base_goal);
        bool finished_within_timeout = move_base_client_.waitForResult(ros::Duration(60.0));

        if (finished_within_timeout) {
            actionlib::SimpleClientGoalState state = move_base_client_.getState();
            ROS_INFO("Current State: %s", state.toString().c_str()); // For debugging
    
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                feedback_.status = "The robot moved to the target pose.";
                as_.publishFeedback(feedback_);
            } else {
                initiateRecoveryRoutine(goal);
            }
        } else {
            initiateRecoveryRoutine(goal);
        }
		scanningRoutine(goal);
    }

    void initiateRecoveryRoutine(const group34_pkg1::NavigateGoalConstPtr &goal) {
        feedback_.status = "The robot failed to move to the target pose. Initiating recovery routine.";
        as_.publishFeedback(feedback_);

        if (!attemptRecovery(goal)) {
            feedback_.status = "Recovery routine failed. Unable to reach target pose.";
        } else {
            feedback_.status = "Recovery routine succeeded. Target pose reached.";
        }
        as_.publishFeedback(feedback_);
    }   

    bool attemptRecovery(const group34_pkg1::NavigateGoalConstPtr &original_goal) {
        static const float recovery_distance = 1.0; // 1 meter
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(60.0); // 60 seconds timeout

        std::vector<tf2::Vector3> recovery_moves = {
            tf2::Vector3(recovery_distance, 0, 0),  // Right
            tf2::Vector3(-recovery_distance, 0, 0), // Left
            tf2::Vector3(0, recovery_distance, 0),  // Forward
            tf2::Vector3(0, -recovery_distance, 0)  // Backward
        };

        for (const auto& move : recovery_moves) {
            if (ros::Time::now() - start_time > timeout) {
                ROS_WARN("Recovery routine timed out.");
                return false; // Exit if recovery process takes too long
            }
            move_base_msgs::MoveBaseGoal recovery_goal;
            recovery_goal.target_pose.header.frame_id = "map";
            recovery_goal.target_pose.header.stamp = ros::Time::now();

            recovery_goal.target_pose.pose.position.x = original_goal->x + move.x();
            recovery_goal.target_pose.pose.position.y = original_goal->y + move.y();
            recovery_goal.target_pose.pose.position.z = original_goal->z + move.z();

            tf2::Quaternion quaternion;
            quaternion.setRPY(original_goal->a, original_goal->b, original_goal->gamma);
            recovery_goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

            move_base_client_.sendGoal(recovery_goal);
            move_base_client_.waitForResult();

            if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                return true; // Recovery successful
            }
        }
        return false; // Recovery failed
    }

	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigate_server");
    NavigateServer server("navigate");
    ros::spin();
    return 0;
}