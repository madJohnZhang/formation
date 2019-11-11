<<<<<<< HEAD
#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>

#include"uav_tracking/boundingbox.h"
=======
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "uav_tracking/boundingbox.h"
#include "uav_tracking/posvel.h"
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc

//using namespace ros;
using namespace std;

<<<<<<< HEAD
class Detector{
=======
class Detector
{
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc
private:
	static vector<int> bbox;
	ros::NodeHandle nh;
	static bool bbox_get;
	sensor_msgs::ImagePtr msg;
	image_transport::Publisher pub;
	ros::Subscriber sub;
<<<<<<< HEAD
	static void bboxCallback(uav_tracking::boundingbox b_box){
		bbox[0]=b_box.x;
		bbox[1]=b_box.y;
		bbox[2]=b_box.w;
		bbox[3]=b_box.h;
		bbox_get=true;
=======
	static void bboxCallback(uav_tracking::boundingbox b_box)
	{
		bbox[0] = b_box.x;
		bbox[1] = b_box.y;
		bbox[2] = b_box.w;
		bbox[3] = b_box.h;
		bbox_get = true;
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc
		//cout<<"bbox get"<<endl;
	}

public:
<<<<<<< HEAD
	Detector(int argc, char **argv){
=======
	Detector(int argc, char **argv)
	{
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc
		ros::init(argc, argv, "detector");
		image_transport::ImageTransport imtr(nh);
		pub = imtr.advertise("livevideo", 1);
		sub = nh.subscribe("boundingbox", 1, bboxCallback);
<<<<<<< HEAD
		bbox_get=false;
		bbox.resize(4);
	}
	bool find(cv::Mat image){
		msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    		pub.publish(msg);
		//cout<<"publish img"<<endl;
    		ros::spinOnce();
		if(bbox_get){
			bbox_get=false;
=======
		bbox_get = false;
		bbox.resize(4);
	}
	bool find(cv::Mat image)
	{
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		pub.publish(msg);
		//cout<<"publish img"<<endl;
		ros::spinOnce();
		if (bbox_get)
		{
			bbox_get = false;
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc
			return true;
		}
		return false;
	}
<<<<<<< HEAD
	vector<int> bounding_box(){
		return bbox;
	}
};
vector<int> Detector::bbox={0,0,0,0};
bool Detector::bbox_get=false;
=======
	vector<int> bounding_box()
	{
		return bbox;
	}
};
vector<int> Detector::bbox = {0, 0, 0, 0};
bool Detector::bbox_get = false;
>>>>>>> 4100612f72981e6b9f0e3af05e057dbbc44673cc
