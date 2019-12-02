#pragma once
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>

#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_broadcast.hpp>

#include "dji_linux_helpers.cpp"
#include "state.cpp"
#include "posEstimate.cpp"

#include "uav_tracking/posvel.h"
#include "uav_tracking/packs.h"

#define PI 3.14159265
#define AUTHORITY "obtainAuthority"

#define SERIES 5

using namespace std;
using namespace cv;

class Formation
{
private:
	FileStorage fs;
	mutex mtxqueue;
	int queuesize;

	State self;
	unordered_map<int, State> others;
	serial::Serial ser;

	int formationNum;
	int seq;
	int8_t synch;

	Mat coefficients;
	Mat topo;
	Mat f;

	posEstimate posE;

	float yaw;
	double *caliGPS;
	uint8_t *dataIn;
	uint8_t *dataOut;
	vector<vector<double>> tmpInit;
	//double *tmpInit;

private:
	const static int sPACKAGESIZE = sizeof(int) + sizeof(float) * 5;

public:
	Formation(int argc, char **argv);
	~Formation();

	void init();

	void getStates(Mat &target);

	Mat getInput();

	void packCallback(const uav_tracking::packs &input);

	static int count;
};

int Formation::count = 0;

void Formation::packCallback(const uav_tracking::packs &input)
{
	State tmp;
	//mtxqueue.lock();
	yaw = input.yaw;
	for (auto i = input.pack.begin(); i != input.pack.end(); i++)
	{
		tmp.number = i->number;
		tmp.yaw = i->yaw;
		tmp.posVel.at<float>(0) = i->x;
		tmp.posVel.at<float>(1) = i->vx;
		tmp.posVel.at<float>(2) = i->y;
		tmp.posVel.at<float>(3) = i->vy;
		if (tmp.number == seq)
		{
			self = tmp;
			synch = i->synch;
		}
		else
		{
			others[tmp.number] = tmp;
		}
	}
	if (others.size() == formationNum - 1)
	{
		queuesize++;
	}
	//mtxqueue.unlock();
}

Formation::Formation(int argc, char **argv)
{
	queuesize = 0;
	fs.open("formation.xml", FileStorage::READ);
	fs["formationNum"] >> formationNum;
	fs["seq"] >> seq;
	fs["topology"] >> topo;
	fs["coefficients"] >> coefficients;
	fs["formation"] >> f;
	topo.convertTo(topo, CV_32FC1);
	coefficients.convertTo(coefficients, CV_32FC1);
	f.convertTo(f, CV_32FC1);
	string port;
	int baudrate;
	serial::Timeout timeout(100);
	fs["port"] >> port;
	fs["baudrate"] >> baudrate;
	cout << "and here" << endl;
	ser.setTimeout(timeout);
	ser.setPort(port);
	ser.setBaudrate(baudrate);
	ser.open();
	ser.flushInput();
	ser.flushOutput();
	fs.release();

	dataOut = new uint8_t[sPACKAGESIZE];
	dataIn = new uint8_t[sPACKAGESIZE * (formationNum - 1)];
	caliGPS = new double[2];
	for (int i = 0; i < SERIES; i++)
	{
		tmpInit.push_back(vector<double>(3 * NODE, 0));
	}
	//tmpInit = new double[3 * NODE * SERIES];
	//memset(tmpInit, 0, 3 * sizeof(double) * NODE * SERIES);

	double LATITUDE_CALI = 180.0 / PI * 111000 * cos(23 * PI / 180);
	double LONGITUDE_CALI = 180.0 / PI * 111000;
}

Formation::~Formation()
{
	delete[] dataOut;
	delete[] dataIn;
	delete[] caliGPS;
}

void Formation::init()
{
	if (queuesize == 0)
	{
		return;
	}
	//judge if all the agents in the network ready
	int count = 0;
	int8_t tmpSynch = synch;
	while (tmpSynch != 0)
	{
		tmpSynch &= tmpSynch - 1;
		count++;
	}
	if (count < formationNum)
	{
		cout << "not enough ready agents" << endl;
		return;
	}
	//size of data at one moment
	vector<double> tmp(3 * NODE, 0.);
	tmp[3 * (self.number - 1)] = self.posVel.at<double>(0);
	tmp[3 * (self.number - 1) + 1] = self.posVel.at<double>(2);
	tmp[3 * (self.number - 1) + 2] = self.yaw;
	if (others.size() == (formationNum - 1))
	{
		for (auto i : others)
		{
			tmp[3 * (i.first - 1)] = i.second.posVel.at<double>(0);
			tmp[3 * (i.first - 1) + 1] = i.second.posVel.at<double>(2);
			tmp[3 * (i.first - 1) + 2] = i.second.yaw;
		}
		if (count < tmpInit.size())
		{
			tmpInit[count].assign(tmp.begin(), tmp.end());
		}
		cout << "init ready to" << endl;
		count++;
	}
	queuesize--;
	if (count == SERIES)
	{
		posE.init(tmpInit, SERIES);
	}
}

/*
	the yaw is in radian
*/

Mat Formation::getInput()
{
	Mat ui = Mat::zeros(2, 1, CV_32FC1);
	Mat result = Mat::zeros(2, 1, CV_32FC1);
	Mat target = Mat::zeros(NODE, 3, CV_64FC1);

	if (queuesize > 0 && others.size() == formationNum - 1)
	{
		for (auto info : others)
		{
			ui += coefficients * ((self.posVel - f.col(self.number - 1)) - (info.second.posVel - f.col(info.second.number - 1)));
		}
		getStates(target);
		Scalar pos = posE.position(target);
		Scalar v = posE.velocity();
		cout << "got position" << pos << endl;
		cout << "got vel: " << v << endl;
		Mat tarPosvel(pos + v);
		tarPosvel.convertTo(tarPosvel, CV_32FC1);
		ui += coefficients * (self.posVel - f.col(self.number - 1) - tarPosvel);
		result.at<float>(0) = ui.at<float>(0) * (float)cos(yaw) + ui.at<float>(1) * (float)sin(yaw);
		result.at<float>(1) = ui.at<float>(1) * (float)cos(yaw) - ui.at<float>(0) * (float)sin(yaw);
		queuesize--;
	}
	return result;
}

void Formation::getStates(Mat &target)
{
	target.at<double>(0, 0) = self.posVel.at<float>(0);
	target.at<double>(0, 1) = self.posVel.at<float>(2);
	target.at<double>(0, 2) = self.yaw;
	int i = 1;
	for (auto node : others)
	{
		target.at<double>(i, 0) = node.second.posVel.at<float>(0);
		target.at<double>(i, 1) = node.second.posVel.at<float>(2);
		target.at<double>(i, 2) = node.second.yaw;
		i++;
	}
}
