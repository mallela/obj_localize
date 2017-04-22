#ifndef OBJDETECT_H
#define OBJDETECT_H

#include <ros/ros.h>
// #include <std_msgs/Int8.h>
//#include <sensor_msgs/ImageConstPtr.h>

#define PI 3.14159265

class ObjDetectLoc{
private:
	ros::Subscriber m_imageSub;
	ros::Subscriber m_depth; 
	ros::Subscriber m_imageDepthSub; 
	// ros::Publisher m_orientationPub;
	ros::Publisher m_objXY;
	// ros::Publisher m_imagePub;
	ros::Publisher tfPub;
	// void flagReceiveCallback(std_msgs::Int8 inFlag);
	void findObj(const sensor_msgs::ImageConstPtr& img);
	void findDepth(const sensor_msgs::PointCloud2& inPC);
	void findDethFromDepthIm(const sensor_msgs::ImageConstPtr& img);
	// void transformCallBack(const geometry_msgs::Point pos);
public:
	ObjDetectLoc(ros::NodeHandle nh);
	geometry_msgs::Point tf;
	
};
#endif
