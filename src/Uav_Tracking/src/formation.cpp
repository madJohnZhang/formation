#pragma once
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <queue>

#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_broadcast.hpp>

#include "dji_linux_helpers.cpp"
#include "state.cpp"
#include "posEstimate.cpp"

#include "uav_tracking/posvel.h"
#include "uav_tracking/packs.h"
#include "uav_tracking/packsDec.h"
#include "uav_tracking/posvelDec.h"

#define PI 3.14159265
#define AUTHORITY "obtainAuthority"

#define SERIES 5
#define TIMEINTEGRAL 0.1

using namespace std;
using namespace cv;

class Formation
{
private:
	FileStorage fs;
	mutex mtxqueue;
	int queuesize;
	int decFlag;
	bool newxy;

	State self;
	unordered_map<int, State> others;

	int formationNum;
	int seq;
	int8_t synch;

	Mat coefficients;
	Mat topo;
	Mat f;

	posEstimate posE;
	posEstimateDec posEDec;

	vector<int> waitXYnum; //nodes number-1 saved
	vector<queue<pair<double, double>>> qotherEstimate;
	Mat xysDec;

	float yaw;
	double *caliGPS;
	uint8_t *dataIn;
	uint8_t *dataOut;
	vector<vector<double>> tmpInit;
	//double *tmpInit;

	VideoWriter formationFigure;

private:
	const static int sPACKAGESIZE = sizeof(int) + sizeof(float) * 5;
	const static int sDECPACKAGESIZE = sPACKAGESIZE + sizeof(int) * 2;

public:
	Formation(int argc, char **argv);
	~Formation();

	void init();
	void initC(); //centralized initialization
	void initD(); //decentralized initialization

	void getStates(Mat &target);

	bool isXYready();
	Mat getInput();

	void packCallback(const uav_tracking::packs &input);
	void packDecCallback(const uav_tracking::packsDec &input);
	void xyDecPub(ros::Publisher &xyDec);
	static int count;
};

int Formation::count = 0;

void Formation::packCallback(const uav_tracking::packs &input)
{
	State tmp;
	//mtxqueue.lock();
	yaw = input.yaw;
	synch = input.synch;
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

void Formation::packDecCallback(const uav_tracking::packsDec &input)
{
	State tmp;
	//mtxqueue.lock();
	yaw = input.yaw;
	synch = input.synch;
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
		}
		else
		{
			if (i->synchXY)
			{
				pair<double, double> otherEstimatePair;
				otherEstimatePair.first = i->esX;
				otherEstimatePair.second = i->esY;
				qotherEstimate[tmp.number - 1].push(otherEstimatePair);
			}

			others[tmp.number] = tmp;
		}
	}
	if (others.size() == formationNum - 1)
	{
		queuesize++;
	}
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
	fs.release();
	caliGPS = new double[2];
	if (!strcmp(argv[1], "dec"))
	{
		posEDec.setTopoNum(seq, topo);
		decFlag = 1;
		dataOut = new uint8_t[sDECPACKAGESIZE];
		dataIn = new uint8_t[sDECPACKAGESIZE * (formationNum - 1)];
		qotherEstimate.resize(formationNum);
		xysDec = Mat::zeros(formationNum, 2, CV_64FC1);
		newxy = false;
		for (int i = 1; i < formationNum; i++)
		{
			if (topo.at<float>(seq - 1, i) != 0)
			{
				waitXYnum.push_back(i);
			}
		}
	}
	else
	{
		decFlag = 0;
		dataOut = new uint8_t[sPACKAGESIZE];
		dataIn = new uint8_t[sPACKAGESIZE * (formationNum - 1)];

		for (int i = 0; i < SERIES; i++)
		{
			tmpInit.push_back(vector<double>(3 * NODE, 0));
		}
	}

	formationFigure.open("/home/sustec/tracking/images/formation.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(1000, 1000));
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
	int countAgent = 0;
	int8_t tmpSynch = synch;
	cout << "synch is: " << synch << endl;
	while (tmpSynch != 0)
	{
		tmpSynch &= tmpSynch - 1;
		countAgent++;
	}
	cout << "countAgent" << countAgent << endl;
	if (countAgent < formationNum)
	{
		cout << "not enough ready agents" << endl;
		return;
	}
	if (decFlag)
	{
		if (count == 0)
		{
			initD();
		}
		else
		{
			getInput();
			count++;
		}
	}
	else
	{
		initC();
	}
}
void Formation::initD()
{
	Mat tmps(1, 3, CV_64FC1);
	tmps.at<double>(0) = self.posVel.at<float>(0);
	tmps.at<double>(1) = self.posVel.at<float>(2);
	tmps.at<double>(2) = self.yaw;
	Mat xys = Mat::zeros(formationNum, 2, CV_64FC1);
	posEDec.position(tmps, xys, 1);
	count = 2;
	newxy = true;
}
void Formation::initC()
{

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
			cout << "init ready to" << endl;
		}
		cout << "init ready to" << endl;
		count++;
		cout << "count is " << count << "------------------------" << endl;
	}
	queuesize--;
	if (count == SERIES)
	{
		posE.init(tmpInit, SERIES);
	}
}

bool Formation::isXYready()
{
	bool isready = true;
	for (auto i : waitXYnum)
	{
		if (qotherEstimate[i].empty())
		{
			isready = false;
			break;
		}
	}
	return isready;
}

void Formation::xyDecPub(ros::Publisher &xyDec)
{
	if (decFlag)
	{
		if (newxy)
		{
			Scalar tmp = posEDec.position();
			uav_tracking::xyDectralize pubXY;
			pubXY.x = tmp[0];
			pubXY.y = tmp[2];
			xyDec.publish(pubXY);
			newxy = false;
		}
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
	Mat figure(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
	Scalar black(0, 0, 0);
	Point x1(499, 0), x2(499, 999), y1(0, 499), y2(999, 499);
	arrowedLine(figure, x2, x1, black, 2, 8, 0, 0.02);
	arrowedLine(figure, y1, y2, black, 2, 8, 0, 0.02);
	circle(figure, {499, 499}, 2, black, 2);
	if (others.size() == formationNum - 1)
	{
		for (auto info : others)
		{
			ui += coefficients * ((self.posVel - f.col(self.number - 1)) - (info.second.posVel - f.col(info.second.number - 1)));
			circle(figure, {(int)info.second.posVel.at<float>(0) * 50 + 499, (int)info.second.posVel.at<float>(2) * 50 + 499}, 5, Scalar(0, 255, 0), 5);
		}
		circle(figure, {(int)self.posVel.at<float>(0) * 50 + 499, (int)self.posVel.at<float>(2) * 50 + 499}, 5, Scalar(255, 150, 0), 5);
		Scalar pos;
		Scalar v;
		if (decFlag)
		{
			Scalar state(self.posVel.at<float>(0), self.posVel.at<float>(2), self.yaw);
			cout << "state" << state << endl;
			Mat selfstate(state);
			cout << "selfstate" << selfstate << endl;
			if (isXYready())
			{
				for (auto index : waitXYnum)
				{
					pair<double, double> tmp = qotherEstimate[index].front();
					qotherEstimate[index].pop();
					xysDec.at<double>(index, 0) = tmp.first;
					xysDec.at<double>(index, 1) = tmp.second;
				}
				pos = posEDec.position(selfstate.t(), xysDec, count);
				newxy = true;
			}
			else
			{
				pos = posEDec.position();
			}

			v = posEDec.velocity();
		}
		else
		{
			getStates(target);
			pos = posE.position(target);
			v = posE.velocity();
		}
		circle(figure, {(int)pos[0] * 50 + 499, (int)pos[2] * 50 + 499}, 5, Scalar(255, 0, 0), 5);
		cout << "got position" << pos << endl;
		cout << "got vel: " << v << endl;
		Mat tarPosvel(pos + v);
		tarPosvel.convertTo(tarPosvel, CV_32FC1);
		ui += coefficients * (self.posVel - f.col(self.number - 1) - tarPosvel);
		//add velocity test
		ui.at<float>(0) *= TIMEINTEGRAL;
		ui.at<float>(0) += self.posVel.at<float>(1);
		ui.at<float>(1) *= TIMEINTEGRAL;
		ui.at<float>(1) += self.posVel.at<float>(3);
		//end
		result.at<float>(0) = ui.at<float>(0) * (float)cos(yaw) + ui.at<float>(1) * (float)sin(yaw);
		result.at<float>(1) = ui.at<float>(1) * (float)cos(yaw) - ui.at<float>(0) * (float)sin(yaw);
		formationFigure << figure;
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
