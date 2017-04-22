#ifndef OBJDETECT_H
#define OBJDETECT_H

#include <ros/ros.h>
// #include <std_msgs/Int8.h>
//#include <sensor_msgs/ImageConstPtr.h>

#define PI 3.14159265

class ObjDetectLoc{
private:
	ros::Subscriber m_imageSub;
	// ros::Subscriber m_goFlag; 
	// ros::Publisher m_orientationPub;
	// ros::Publisher m_xyzPlacePub;
	ros::Publisher m_imagePub;
	ros::Subscriber tf_sub;
	// void flagReceiveCallback(std_msgs::Int8 inFlag);
	void findObj(const sensor_msgs::ImageConstPtr& img);
	// void transformCallBack(const geometry_msgs::Point pos);
public:
	ObjDetectLoc(ros::NodeHandle nh);
	geometry_msgs::Point tf;
	
};
#endif
