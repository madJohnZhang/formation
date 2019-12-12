#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "uav_tracking/boundingbox.h"
#include "uav_tracking/posvel.h"

//using namespace ros;
using namespace std;

class Detector
{
private:
	static vector<int> bbox;
	ros::NodeHandle nh;
	static bool bbox_get;
	sensor_msgs::ImagePtr msg;
	image_transport::Publisher pub;
	ros::Subscriber sub;
	static void bboxCallback(uav_tracking::boundingbox b_box)
	{
		bbox[0] = b_box.x;
		bbox[1] = b_box.y;
		bbox[2] = b_box.w;
		bbox[3] = b_box.h;
		bbox_get = true;
		//cout<<"bbox get"<<endl;
	}

public:
	Detector(int argc, char **argv)
	{
		ros::init(argc, argv, "detector");
		image_transport::ImageTransport imtr(nh);
		pub = imtr.advertise("livevideo", 1);
		sub = nh.subscribe("boundingbox", 1, bboxCallback);
		bbox_get = false;
		bbox.resize(4);
	}
	bool find(cv::Rect b_box)
	{
		bbox[0] = b_box.x;
		bbox[1] = b_box.y;
		bbox[2] = b_box.width;
		bbox[3] = b_box.height;
		return true;
	}
	vector<int> bounding_box()
	{
		return bbox;
	}
};
vector<int> Detector::bbox = {0, 0, 0, 0};
bool Detector::bbox_get = false;
