#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include "send_goal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

SendGoal::SendGoal(ros::NodeHandle nh){
m_locSub = nh.subscribe("/object_detect/location", 5, &SendGoal::moveToGoal, this);
}

void SendGoal::moveToGoal(const geometry_msgs::PointStamped &loc3d)
{

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	// actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	MoveBaseClient ac("move_base", true);
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  loc3d.point.x;
	goal.target_pose.pose.position.y =  loc3d.point.y;
	goal.target_pose.pose.position.z =  0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
	}
	return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
  SendGoal goal(n);
	ros::spin();
}
