#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/topic.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <group34_pkg2/ReadTagsAction.h>
#include <string>
#include <vector>
#include <cctype>
#include <map>
#include <cstdlib>
#include <sstream>


#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>




class ReadTagsServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<group34_pkg2::ReadTagsAction> as_;
    std::string action_name_;
    group34_pkg2::ReadTagsFeedback feedback_;
    group34_pkg2::ReadTagsResult result_;

public:
    ReadTagsServer(std::string name)
        : as_(nh_, name, boost::bind(&ReadTagsServer::executeCB, this, _1), false),
          action_name_(name)
	{	
        as_.start();
    }
	
		
    void executeCB(const group34_pkg2::ReadTagsGoalConstPtr &goal) {
		
		result_.obstacles_coords.clear();
		result_.objects_coords.clear();

		ROS_INFO("Received Goal Id: %d", goal->id);
		ros::Duration(1).sleep(); 
		//Reads the detected objects from the /tag_detections topic
  		apriltag_ros::AprilTagDetectionArrayConstPtr from_tag_detections;
  		apriltag_ros::AprilTagDetectionArray msg;
  		from_tag_detections = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
  		if(from_tag_detections != NULL){
    		msg = *from_tag_detections;
  		}

  		// Create TF listener
		tf2_ros::Buffer tfBuffer;
  		tf2_ros::TransformListener tfListener(tfBuffer);

    	// Transform pose from camera frame to robot frame using TF
    	geometry_msgs::TransformStamped transformStamped;
		ros::Duration(1).sleep();

  		// Get poses of the objects on the pick-up table
  		for (auto detection : msg.detections) {
    		// Identify the markers through the IDs encoded in the QR codes
    		int id = detection.id[0];
    		try{
      			transformStamped = tfBuffer.lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0));
   			}
    		catch (tf2::TransformException &ex) {
      			ROS_WARN("%s",ex.what());
      			ros::Duration(1.0).sleep();
      			continue;
    		}

    		// Get the pose of the marker in the camera frame
    		geometry_msgs::Pose tag_pose = detection.pose.pose.pose;
			geometry_msgs::PoseStamped camera_pose, robot_pose_stamped;
			camera_pose.pose = tag_pose;
			camera_pose.header.frame_id = "xtion_rgb_optical_frame";
			robot_pose_stamped.header.frame_id = "base_footprint";

			tf2::doTransform(camera_pose, robot_pose_stamped, transformStamped);
			
			//PUT THE RESULT OF THE READINGS HERE
			double x, y, z; 
			x = robot_pose_stamped.pose.position.x;
			y = robot_pose_stamped.pose.position.y;
			z = robot_pose_stamped.pose.position.z;
			
			if (id == goal->id) {
  				result_.objects_coords.push_back(x);
				result_.objects_coords.push_back(y);
				result_.objects_coords.push_back(z);
			} else if ( id >= 4) {
			  	result_.obstacles_coords.push_back(x);
				result_.obstacles_coords.push_back(y);
			}

			/*
    		// Convert pose to transform
    		tf2::Transform camera_to_robot_transform = transformStamped.inverse();

    		// Apply the transform to the camera pose
   		 	tf2::Transform transformed_pose = camera_to_robot_transform * camera_pose;

   		 	// Compute the pose of the object in the robot frame
   		 	geometry_msgs::Pose robot_pose;
    	 	robot_pose.position.x = transformed_pose.getOrigin().getX();
   		 	robot_pose.position.y = transformed_pose.getOrigin().getY();
   		 	robot_pose.position.z = transformed_pose.getOrigin().getZ();
			*/

			/* Lorenzon Solution
 			// Convert geometry_msgs::TransformStamped to tf2::Transform
            tf2::Transform camera_to_robot_transform;
            tf2::fromMsg(transformStamped.transform, camera_to_robot_transform);

            // Apply the transform to the camera pose
            tf2::Transform transformed_pose = camera_to_robot_transform * camera_pose_tf2;

            // Convert the transformed pose (tf2::Transform) to a geometry_msgs::Transform
            geometry_msgs::Transform transformed_msg = tf2::toMsg(transformed_pose);
			*/	
			
			/*
			ROS_INFO("fram id: %s", robot_pose_stamped.header.frame_id.c_str());
			ROS_INFO("x transformed: %f", robot_pose_stamped.pose.position.x);
			*/
			ROS_INFO("z transformed: %f", robot_pose_stamped.pose.position.z);

		} 		
		//result_.status = "FAILED";
		//as_.setAborted(result_);
		result_.status = "SUCCEEDED";
		as_.setSucceeded(result_);	
    }
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_tags_server");
    ReadTagsServer server("read_tags");
    ros::spin();
    return 0;
}
