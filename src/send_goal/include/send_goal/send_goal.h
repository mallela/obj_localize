#ifndef SENDGOAL_H
#define SENDGOAL_H

#include <ros/ros.h>


class SendGoal{
private:
	ros::Subscriber m_locSub;
  void moveToGoal(const geometry_msgs::PointStamped &loc3d);

public:
	SendGoal(ros::NodeHandle nh);
};
#endif
