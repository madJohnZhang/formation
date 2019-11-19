#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <opencv2/opencv.hpp>

class State
{
public:
	int number;
	float yaw; //yaw in tangent
	Mat posVel;
	State()
	{
		number = 0;
		posVel = Mat(4, 1, CV_32FC1);
	}
	State &operator=(State &source)
	{
		number = source.number;
		yaw = source.yaw;
		posVel = source.posVel.clone();
		return *this;
	}
};

#endif
