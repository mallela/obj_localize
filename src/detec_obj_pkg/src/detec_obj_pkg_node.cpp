// 
#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
// #include <math.h> /* tan */
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Int64.h>
// #include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "detec_obj_pkg_node.h" 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

int x=0, y=0, h=0, w=0;
sensor_msgs::PointCloud2 globalPC;
// float depth =0;
// std_msgs::Int8 flag;
// flag.data = 0;

ObjDetectLoc::ObjDetectLoc(ros::NodeHandle nh){
	m_objXY = nh.advertise<geometry_msgs:: Point> ("/object_detect/xy",100, true);
	tfPub = nh.advertise<geometry_msgs::PointStamped> ("/object_detect/location",100, true);

	// tf_sub = nh.subscribe("irb120/transform", 5, &ObjDetectLoc::transformCallBack,this);
	m_imageSub = nh.subscribe("/camera/rgb/image_raw", 5, &ObjDetectLoc::findObj, this);
	// m_imageDepthSub = nh.subscribe("/camera/depth/image_raw", 5, &ObjDetectLoc::findDethFromDepthIm, this);
	m_depth = nh.subscribe("/camera/depth/points", 5, &ObjDetectLoc::findDepth, this);
}
// void ObjDetectLoc::findDethFromDepthIm(const sensor_msgs::ImageConstPtr& imgD){
// 	cv_bridge::CvImagePtr inMsgPtr;
// 	inMsgPtr = cv_bridge::toCvCopy(imgD,sensor_msgs::image_encodings::BGR8);
// 	float dist_val = inMsgPtr->image.at<float>( x,y );
// 	cout<<dist_val<<endl;
// }
void ObjDetectLoc::findDepth(const sensor_msgs::PointCloud2& inPC)
{

	pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
	pcl::fromROSMsg(inPC,PointCloudXYZ);


	h = inPC.height;
	w = inPC.width;
	// int cloudsize = h*w;
	std::string frame = "objframe";
    tf::Transform transform;
    // transform.clear();
    static tf::TransformBroadcaster br;
	tf::TransformListener listener;
    tf::Quaternion q;
	// depth = inPC.data[x+y*w];
	if (!(isnan(PointCloudXYZ.points[x+y*w].x) && isnan(PointCloudXYZ.points[x+y*w].y) && isnan(PointCloudXYZ.points[x+y*w].z))){


		geometry_msgs::PointStamped pt;
		geometry_msgs::PointStamped pt_transformed;
		pt.header.frame_id = "map";
		pt.header.stamp = ros::Time();
		pt.point.x = PointCloudXYZ.points[x+y*w].x;
		pt.point.y = PointCloudXYZ.points[x+y*w].y;
		pt.point.z = PointCloudXYZ.points[x+y*w].z;

		ros::Rate rate(1000);
		while(ros::ok()){
	 try{
            listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::spinOnce();
            continue;
        }

		
		// try{
		// listener.transformPoint("odom", pt, pt_transformed);
		// tfPub.publish(pt_transformed);
		// } catch(tf::TransformException ex){
		// 	ROS_ERROR("%s",ex.what());
		// }
		transform.setOrigin( tf::Vector3(pt.point.x , pt.point.y, pt.point.z) );
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", frame));
		rate.sleep();
		cout<<PointCloudXYZ.points[x+y*w]<<endl;
		ros::spinOnce();
	}
}
	// PointCloudXYZ.clear();
	// cout<< depth<<endl;


}
void ObjDetectLoc::findObj(const sensor_msgs::ImageConstPtr& img){
	static int flag = 0;
	cv::Mat inImage;
	cv::Mat imHSV, imGray, imFiltered, imRedObj, mask1, mask2, dummyMat;

	cv_bridge::CvImagePtr inMsgPtr;
	geometry_msgs::Point coordsObj;
	coordsObj.x=0;
	coordsObj.y=0;
	float depth=0;
	inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
	inImage = inMsgPtr->image;
	
	cvtColor(inImage, imHSV, cv::COLOR_BGR2HSV);
	int sensitivity = 55;
	cv::inRange(imHSV, cv::Scalar(60 - sensitivity, 100, 100), cv::Scalar(60+sensitivity, 255, 255), mask1);
	// cv::inRange(imHSV, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
	// cv::inRange(imHSV, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
	// bitwise_and(mask1, mask2, mask1);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy ;	
	dummyMat = mask1.clone();

	findContours(dummyMat,contours,hierarchy, RETR_TREE,CHAIN_APPROX_NONE,Point(0,0));
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );	
	// cout<<refx << " "<< refy<< " "<<endl;
	for(int i = 0; i< contours.size(); i++){
	    Moments moment = moments((cv::Mat)contours[i]);
	    if (moment.m00!=0 ){
	        // merge(frame, 3, frame);
	        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
	        boundRect[i] = boundingRect(Mat(contours_poly[i]));
	        rectangle( inImage, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 1, 8, 0 );
	        coordsObj.x = moment.m10/moment.m00;
	        coordsObj.y = moment.m01/moment.m00;
	        x = coordsObj.x; y = coordsObj.y;
	        m_objXY.publish(coordsObj);
	        // depth =globalPC.data[x+y*w];
	        // cout<< depth<<endl;
	        break;
	    }

	}  


	cv::imshow("red obj", mask1);
	cv::imshow("bounding box", inImage);

	cv::waitKey(1);
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "chip_place_node");
	ros::NodeHandle nh;	
	ObjDetectLoc place(nh);
	ros::spin();
}