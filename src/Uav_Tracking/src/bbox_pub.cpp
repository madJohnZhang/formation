#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include<cv_bridge/cv_bridge.h>

#include"uav_tracking/boundingbox.h"

using namespace std;
using namespace cv;

Mat image;
static vector<int> boundingBox(4);
static bool selectObject = false;
static bool startSelection = false;

Point origin;
Rect choose;
bool select_flag = false, standby =true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image=cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat tmp = image.clone();
    if (select_flag)
    {
	rectangle(tmp, choose, Scalar(0, 0, 255));
    }
    imshow("view", tmp);
    waitKey(1);
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void mouseCallback(int event, int x, int y, int flags, void * userdata)
{
	if (!standby)
	{
		return;
	}
	if (select_flag)
	{
		choose.x = MIN(origin.x, x);
		choose.y = MIN(origin.y, y);
		choose.width = abs(origin.x - x);
		choose.height = abs(origin.y - y);
		choose &= Rect(0, 0, 640, 480);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		select_flag = true;
		origin = Point(x, y);
		cout << x << y << endl;
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		standby = false;
		select_flag = false;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  waitKey(1); 
  uav_tracking::boundingbox bbox;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("livevideo", 1, imageCallback);
  ros::Publisher pub=nh.advertise<uav_tracking::boundingbox>("boundingbox", 1);
  setMouseCallback("view", mouseCallback);
  while(ros::ok()){
    if(!standby){
      bbox.x=choose.x;
      bbox.y=choose.y;
      bbox.w=choose.width;
      bbox.h=choose.height;
      cout<<"publish bbox"<<endl;
      pub.publish(bbox);
      //selectObject=false;
      //startSelection=false;
      usleep(100000);
      exit(1);
    }
    ros::spinOnce();
  }
  cv::destroyWindow("view");
}
