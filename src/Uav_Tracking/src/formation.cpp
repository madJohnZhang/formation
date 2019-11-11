#pragma once
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_broadcast.hpp>

#include "dji_linux_helpers.cpp"
#include "state.cpp"
#include "posEstimate.cpp"

#define PI 3.14159265
#define AUTHORITY "obtainAuthority"

#define SERIES 20

using namespace std;
using namespace cv;

class Formation
{
private:
	FileStorage fs;

	State self;
	map<int, State> others;
	serial::Serial ser;

	int formationNum;
	int seq;

	Mat coefficients;
	Mat topo;
	Mat f;

	LinuxSetup *linuxEnvironment;
	Vehicle *vehicle;

	posEstimate posE;
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
	void initVehicle();
	void sendInfo();
	void getSelf();
	void getInfoFromOthers();
	double getYaw();
	Vehicle *getVehicle();

	void init();
	void cali(int calibration);
	void getStates(Mat &target);
	void print(fstream *self, fstream *xb);
	Mat getInput();
	static int count;
};

int Formation::count = 0;

Formation::Formation(int argc, char **argv)
{
	linuxEnvironment = new LinuxSetup(1, argv);
	vehicle = linuxEnvironment->getVehicle();
	cout << "here" << endl;
	if (vehicle == NULL)
	{
		std::cout << "Vehicle not initialized, exiting.\n";
		exit(-1);
	}

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
	delete linuxEnvironment;
	delete[] dataOut;
	delete[] dataIn;
	delete[] caliGPS;
}

void Formation::init()
{
	getSelf();
	sendInfo();
	getInfoFromOthers();
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
		count++;
	}
	if (count == SERIES)
	{
		posE.init(tmpInit, SERIES);
	}
}

void Formation::initVehicle()
{
	ACK::ErrorCode ack = vehicle->obtainCtrlAuthority(1000);
	if (ACK::getError(ack))
	{
		ACK::getErrorCodeMessage(ack, AUTHORITY);
		exit(-1);
	}
	cout << "authority init complete" << endl;
}

void Formation::getSelf()
{
	/*
	Telemetry::Mag mag = vehicle->broadcast->getMag();
	cout<<"the Mag of the drone is"<<mag.x<<" "<<mag.y<<" "<<mag.z<<endl;
	*/
	double coeX = 180. * 102000 / 3.1416;
	double coeY = 180. * 111000 / 3.1416;

	self.number = seq;
	Telemetry::GlobalPosition gp = vehicle->broadcast->getGlobalPosition();
	double posX = (gp.latitude - caliGPS[0]) * coeX;
	double posY = (gp.longitude - caliGPS[1]) * coeY;
	self.posVel.at<float>(0) = (float)posX;
	self.posVel.at<float>(2) = (float)posY;

	Telemetry::Vector3f v = vehicle->broadcast->getVelocity();
	self.posVel.at<float>(1) = v.x;
	self.posVel.at<float>(3) = v.y;
	self.yaw = (float)tan(getYaw());
	//Telemetry::Quaternion q4 = vehicle->broadcast->getQuaternion();
	//cout << "the yaw of the drone is: " << atan2(2 * (q4.q0 * q4.q3 + q4.q1 * q4.q2), 1 - 2 * (pow(q4.q2, 2) + pow(q4.q3, 2))) << endl;
}

void Formation::sendInfo()
{
	memcpy(dataOut, &self.number, sizeof(int));
	memcpy(dataOut + 4, &self.yaw, sizeof(float));
	memcpy(dataOut + 8, self.posVel.data, sizeof(float) * 4);
	ser.write(dataOut, sPACKAGESIZE);
	cout << "infor sent" << endl;
}

//obtain packages from other nodes
void Formation::getInfoFromOthers()
{
	/*
	 * there are two ways
	 * 1. every time read the info of the corresponding drones
	 * 2. every time load the info of all the drones and calculate
	 *
	 */
	/*
	one possible everytime,every ros::ok(), read two packages. only if obtained all
	the packages calculate the input.
	
	*/

	cout << "bytes read: " << dec << ser.read(dataIn, sPACKAGESIZE * (formationNum - 1));

	int number = 0;
	double yaw = 0;
	for (int i = 0; i < formationNum - 1; i++)
	{
		State state;
		int offset = i * sPACKAGESIZE;
		memcpy(&number, dataIn + offset, sizeof(int));
		state.number = number;
		memcpy(&yaw, dataIn + offset + 4, sizeof(float));
		state.yaw = yaw;
		Mat posVel(4, 1, CV_32FC1, dataIn + offset + 8);
		state.posVel = posVel.clone();
		cout << "data from others" << endl;
		cout << "the number is:" << state.number << endl;
		//cout << posVel << endl;
		if (number >= 1 && number <= formationNum)
		{
			others[number] = state;
		}
	}
}

/*
	the yaw is in radian
*/
double Formation::getYaw()
{
	Telemetry::Quaternion q4 = vehicle->broadcast->getQuaternion();
	double yaw = atan2(2 * (q4.q0 * q4.q3 + q4.q1 * q4.q2), 1 - 2 * (pow(q4.q2, 2) + pow(q4.q3, 2)));
	cout << "yaw is: " << yaw << endl;
	return yaw;
}

Mat Formation::getInput()
{
	getSelf();
	sendInfo();
	getInfoFromOthers();

	Mat ui = Mat::zeros(2, 1, CV_32FC1);
	Mat result = Mat::zeros(2, 1, CV_32FC1);
	Mat target = Mat::zeros(NODE, 3, CV_64FC1);

	if (others.size() == formationNum - 1)
	{
		double yaw = getYaw();
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
	}
	return result;
}

void Formation::cali(int calibration)
{
	if (calibration != 0)
	{
		fstream cali_file_in("cali.txt", ios::in);
		cali_file_in >> caliGPS[0] >> caliGPS[1];

		cout << "cali_file_in" << caliGPS[0] << " " << caliGPS[1];
		cout << "caliGPS info obtained" << endl;
		cali_file_in.close();
		return;
	}
	fstream log_file("caliGPS.csv", ios::trunc | ios::out);
	log_file << "latitude,"
			 << "longitude,"
			 << "yaw" << endl;
	ros::Rate loop_rate(20);
	int flag = 0;
	while (ros::ok() && flag < 100)
	{

		ros::spinOnce();
		loop_rate.sleep();

		Telemetry::GlobalPosition gp = vehicle->broadcast->getGlobalPosition();
		log_file << fixed << setprecision(14) << gp.latitude << ","
				 << gp.longitude << ","
				 << getYaw() << endl;
		flag++;
		if (flag == 70)
		{
			caliGPS[0] = gp.latitude;
			caliGPS[1] = gp.longitude;
			fstream cali_file("cali.txt", ios::trunc | ios::out);
			cali_file << fixed << setprecision(14) << caliGPS[0] << endl
					  << caliGPS[1];
			cali_file.close();
			cout << "caliGPS info obtained" << endl;
		}
	}
}

void Formation::print(fstream *sf, fstream *xb)
{
	*sf << fixed << setprecision(14) << self.posVel.at<float>(0) << "," << self.posVel.at<float>(2) << "," << self.yaw << endl;
	for (auto info : others)
	{
		*xb << fixed << setprecision(14) << info.first << "," << info.second.posVel.at<float>(0) << "," << info.second.posVel.at<float>(2) << "," << info.second.yaw << endl;
	}
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

Vehicle *Formation::getVehicle()
{
	return vehicle;
}
