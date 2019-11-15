#include <iostream>
#include <mutex>
#include <map>
#include <fstream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <serial/serial.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_broadcast.hpp>

#include "dji_linux_helpers.cpp"

#include "uav_tracking/posvel.h"
#include "uav_tracking/controldata.h"
#include "uav_tracking/packs.h"
#include "std_msgs/Float32.h"

using namespace std;
using namespace cv;

#define PI 3.14159265
#define AUTHORITY "obtainAuthority"
#define SERIES 20
#define NODE 3
//to be confirmed:  is data in uav_tracking::posVel struct arraged by order.
class communicator
{
private:
    FileStorage fs;

    serial::Serial ser;

    int formationNum;
    int seq;

    uav_tracking::posvel self;
    map<int, uav_tracking::posvel> others;

    Mat coefficients;
    Mat topo;
    Mat f;

    LinuxSetup *linuxEnvironment;

    float yawAngle;

    double *caliGPS;
    uint8_t *dataIn;
    uint8_t *dataOut;
    vector<vector<double>> tmpInit;

    Vehicle *vehicle;

    fstream dataSelf;
    fstream dataXb;

private:
    const static int sPACKAGESIZE = sizeof(int) + sizeof(float) * 5;

public:
    communicator();
    ~communicator();
    void initVehicle();
    void sendInfo();
    void getSelf();
    double getYaw();
    void getInfoFromOthers();
    Vehicle *getVehicle();

    void cali(int calibration);
    void getStates(Mat &target);
    void print(fstream *self, fstream *xb);
    Mat getInput();

    void print();

    void loop(ros::Publisher &pub);

    void controlCallback(const uav_tracking::controldata &input);

    static int count;
};

Vehicle *communicator::getVehicle()
{
    return vehicle;
}

communicator::communicator()
{
    linuxEnvironment = new LinuxSetup(1, NULL);
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
    ser.setParity(serial::parity_even);

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

    //print data
    dataSelf = fstream("dataSelf.csv", ios::trunc | ios::out);
    dataXb = fstream("dataXb.csv", ios::trunc | ios::out);
    dataSelf << "latitude,"
             << "longitude,"
             << "yaw" << endl;
    dataXb << "number,"
           << "latitude,"
           << "longitude,"
           << "yaw" << endl;
}

communicator::~communicator()
{
    delete linuxEnvironment;
    delete[] dataOut;
    delete[] dataIn;
    delete[] caliGPS;
    vehicle->releaseCtrlAuthority(10);
}

void communicator::initVehicle()
{
    /*ACK::ErrorCode ack = vehicle->obtainCtrlAuthority(1000);
	if (ACK::getError(ack))
	{
		ACK::getErrorCodeMessage(ack, AUTHORITY);
		exit(-1);
	}*/
    cout << "authority init complete" << endl;
}

void communicator::getSelf()
{
    double coeX = 180. * 102000 / 3.1416;
    double coeY = 180. * 111000 / 3.1416;

    self.number = seq;
    Telemetry::GlobalPosition gp = vehicle->broadcast->getGlobalPosition();
    double posX = (gp.latitude - caliGPS[0]) * coeX;
    double posY = (gp.longitude - caliGPS[1]) * coeY;
    self.x = (float)posX;
    self.y = (float)posY;

    Telemetry::Vector3f v = vehicle->broadcast->getVelocity();
    self.vx = v.x;
    self.vy = v.y;
    self.yaw = (float)tan(getYaw());
}

//send info to other agent by XBEE
void communicator::sendInfo()
{
    memcpy(dataOut, &self, sizeof(uav_tracking::posvel));
    ser.write(dataOut, sPACKAGESIZE);
    cout << "infor sent" << endl;
}

void communicator::getInfoFromOthers()
{
    cout << "bytes read: " << ser.read(dataIn, sPACKAGESIZE * (formationNum - 1)) << '\n';

    int number = 0;
    uav_tracking::posvel tmp;
    for (int i = 0; i < formationNum - 1; i++)
    {
        memcpy(&tmp, dataIn + i * sPACKAGESIZE, sizeof(uav_tracking::posvel));
        if (tmp.number >= 1 && tmp.number < formationNum && tmp.x != 0 && tmp.y != 0)
        {
            others[tmp.number] = tmp;
        }
        cout << "the yaw from " << tmp.number << "is " << tmp.yaw << endl;
    }
}

double communicator::getYaw()
{
    Telemetry::Quaternion q4 = vehicle->broadcast->getQuaternion();
    double yaw = atan2(2 * (q4.q0 * q4.q3 + q4.q1 * q4.q2), 1 - 2 * (pow(q4.q2, 2) + pow(q4.q3, 2)));
    cout << "yaw is: " << yaw << endl;
    yawAngle = (float)yaw;
    return yaw;
}

/*
void communicator::print(fstream *sf, fstream *xb)
{
	*sf << fixed << setprecision(14) << self.posVel.at<float>(0) << "," << self.posVel.at<float>(2) << "," << self.yaw << endl;
	for (auto info : others)
	{
		*xb << fixed << setprecision(14) << info.first << "," << info.second.posVel.at<float>(0) << "," << info.second.posVel.at<float>(2) << "," << info.second.yaw << endl;
	}
}*/

void communicator::controlCallback(const uav_tracking::controldata &input)
{
    if (vehicle->broadcast->getRC().gear == -4545)
    {
        Control::CtrlData cd(0x4A, input.vx, input.vy, input.vz, input.vyaw);
        vehicle->control->flightCtrl(cd);
    }
}

void communicator::cali(int calibration)
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

void communicator::loop(ros::Publisher &pub)
{
    uav_tracking::packs tmp;
    tmp.pack.push_back(self);
    for (auto i : others)
    {
        tmp.pack.push_back(i.second);
    }
    tmp.yaw = yawAngle;
    pub.publish(tmp);
}

void communicator::print()
{
    dataSelf << self.x << "," << self.vx << "," << self.y << "," << self.vy << "," << self.yaw << endl;
    for (auto i : others)
    {
        dataXb << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
        cout << "xbee data is: " << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "com");
    ros::NodeHandle nh;
    communicator com;

    int calibration = 1;
    cout << "please input the cali mode: not 0 for calibration completed; 0, do calibration" << endl;
    cin >> calibration;
    com.cali(calibration);
    cout << "cali complete press any button to continue." << endl;
    getchar();

    ros::Publisher pub = nh.advertise<uav_tracking::packs>("posvel_msg", 2);
    ros::Subscriber sub = nh.subscribe("controlData", 1, &communicator::controlCallback, &com);
    ros::Rate loop_rate(15);
    int i = 0;
    while (ros::ok())
    {
        loop_rate.sleep();
        if (i == 0)
        {
            com.getSelf();
            com.sendInfo();
            com.getInfoFromOthers();
            com.loop(pub);
            com.print();
        }
        i++;
        i %= 3;
        ros::spinOnce();
    }
    return 0;
}