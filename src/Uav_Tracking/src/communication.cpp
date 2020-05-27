#include <iostream>
#include <mutex>
#include <map>
#include <fstream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <serial/serial.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_broadcast.hpp>
#include <std_msgs/Int8.h>

#include "dji_linux_helpers.cpp"

#include "uav_tracking/posvel.h"
#include "uav_tracking/controldata.h"
#include "uav_tracking/packs.h"
#include "std_msgs/Float32.h"
#include "uav_tracking/packsDec.h"
#include "uav_tracking/posvelDec.h"
#include "uav_tracking/xyDectralize.h"

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

    int decFlag;

    bool startControl;

    uav_tracking::posvel self;
    uav_tracking::posvelDec selfDec;
    int synch;
    map<int, uav_tracking::posvel> others;
    map<int, uav_tracking::posvelDec> othersDec;

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
    const static int sPACKAGESIZE = sizeof(uav_tracking::posvel);
    const static int sDecPACKAGESIZE = sizeof(uav_tracking::posvelDec);

public:
    communicator(char *argv);
    ~communicator();
    void initVehicle();
    void sendInfo();
    void getSelf();
    double getYaw();
    void getInfoFromOthers();
    Vehicle *getVehicle();

    void cali(int calibration);
    void getStates(Mat &target);
    Mat getInput();

    void print();

    void loop(ros::Publisher &pub, ros::Publisher &pubDec);

    void controlCallback(const uav_tracking::controldata &input);
    void controlTest();
    void readyCalback(const std_msgs::Int8 &ready);
    void xyCallback(const uav_tracking::xyDectralize &input);
    float bound(const float &input);
    static int count;
};

int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

Vehicle *communicator::getVehicle()
{
    return vehicle;
}

communicator::communicator(char *argv)
{
    linuxEnvironment = new LinuxSetup(1, NULL);
    vehicle = linuxEnvironment->getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        exit(-1);
    }
    startControl = false;
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
    ser.setTimeout(timeout);
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    ser.setParity(serial::parity_even);

    ser.open();
    ser.flushInput();
    ser.flushOutput();
    fs.release();

    self.number = seq;
    selfDec.number = seq;
    synch = 0;
    if (strcmp(argv, "dec") == 0)
    {
        decFlag = 1;
    }
    else
    {
        decFlag = 0;
    }
    if (decFlag)
    {
        dataOut = new uint8_t[sDecPACKAGESIZE];
        dataIn = new uint8_t[sDecPACKAGESIZE * (formationNum - 1)];
    }
    else
    {
        dataOut = new uint8_t[sPACKAGESIZE];
        dataIn = new uint8_t[sPACKAGESIZE * (formationNum - 1)];
    }

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
    //dec
    delete linuxEnvironment;
    delete[] dataOut;
    delete[] dataIn;
    delete[] caliGPS;
    ser.close();
    cout << "ser is closed" << endl;
}

void communicator::initVehicle()
{
    /*while (vehicle->broadcast->getRC().mode != 8000)
    {
        cout << "switch the mode to F mode. activate fail-------------" << endl;
        waitKey(500);
    }*/
    ACK::ErrorCode ack = vehicle->obtainCtrlAuthority(1000);
    if (ACK::getError(ack))
    {
        ACK::getErrorCodeMessage(ack, AUTHORITY);
        exit(-1);
    }
    cout << "---------------authority init complete---------------------" << endl;
}

void communicator::getSelf()
{
    double coeX = 180. * 102000 / 3.1416;
    double coeY = 180. * 111000 / 3.1416;

    Telemetry::GlobalPosition gp = vehicle->broadcast->getGlobalPosition();
    double posX = (gp.latitude - caliGPS[0]) * coeX;
    double posY = (gp.longitude - caliGPS[1]) * coeY;
    Telemetry::Vector3f v = vehicle->broadcast->getVelocity();
    if (decFlag)
    {
        selfDec.x = (float)posX;
        selfDec.y = (float)posY;
        selfDec.vx = v.x;
        selfDec.vy = v.y;
        selfDec.yaw = (float)tan(getYaw());

        //temporary
        selfDec.x = (float)2.37295;
        selfDec.y = (float)4.50968;
        selfDec.yaw = (float)-7.14664;
    }
    else
    {
        self.x = (float)posX;
        self.y = (float)posY;

        self.vx = v.x;
        self.vy = v.y;
        self.yaw = (float)tan(getYaw());
    }
}

//send info to other agent by XBEE
void communicator::sendInfo()
{
    if (decFlag)
    {
        uav_tracking::posvelDec sendTmpDec = selfDec;
        sendTmpDec.number |= (synch << 8);
        memcpy(dataOut, &sendTmpDec, sDecPACKAGESIZE);
        ser.write(dataOut, sDecPACKAGESIZE);
    }
    else
    {
        uav_tracking::posvel sendTmp = self;
        sendTmp.number |= (synch << 8);
        memcpy(dataOut, &sendTmp, sizeof(uav_tracking::posvel));
        ser.write(dataOut, sPACKAGESIZE);
    }
    cout << "infor sent" << endl;
}

void communicator::getInfoFromOthers()
{
    if (decFlag)
    {
        cout << "bytes read: " << ser.read(dataIn, sDecPACKAGESIZE * (formationNum - 1)) << '\n';

        int number = 0;
        uav_tracking::posvelDec tmp;
        for (int i = 0; i < formationNum - 1; i++)
        {
            memcpy(&tmp, dataIn + i * sDecPACKAGESIZE, sDecPACKAGESIZE);
            int otherSynch = (tmp.number & 0xFF00) >> 8;
            tmp.number &= 0x00FF;
            if (tmp.number >= 1 && tmp.number <= formationNum && tmp.x != 0 && tmp.y != 0)
            {
                synch |= otherSynch;
                othersDec[tmp.number] = tmp;
            }
            cout << "the yaw from " << tmp.number << "is " << tmp.x << " " << tmp.y << " " << tmp.yaw << endl;
        }
    }
    else
    {
        cout << "bytes read: " << ser.read(dataIn, sPACKAGESIZE * (formationNum - 1)) << '\n';

        int number = 0;
        uav_tracking::posvel tmp;
        for (int i = 0; i < formationNum - 1; i++)
        {
            memcpy(&tmp, dataIn + i * sPACKAGESIZE, sizeof(uav_tracking::posvel));
            int otherSynch = (tmp.number & 0xFF00) >> 8;
            tmp.number &= 0x00FF;
            if (tmp.number >= 1 && tmp.number <= formationNum && tmp.x != 0 && tmp.y != 0)
            {
                synch |= otherSynch;
                others[tmp.number] = tmp;
            }
            cout << "the yaw from " << tmp.number << "is " << tmp.x << " " << tmp.y << " " << tmp.yaw << endl;
        }
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
float communicator::bound(const float &input)
{
    if (input > 1)
    {
        return 1;
    }
    else if (input < -1)
    {
        return -1;
    }
    else
    {
        return input;
    }
}

void communicator::xyCallback(const uav_tracking::xyDectralize &input)
{
    selfDec.esX = input.x;
    selfDec.esY = input.y;
}

void communicator::controlCallback(const uav_tracking::controldata &input)
{
    if (startControl == false)
    {
        initVehicle();
        startControl = true;
    }
    if (vehicle->broadcast->getRC().gear == -4545)
    {

        /*
    *** test Control::StableEnable 
    */
        Control::CtrlData cd(0x4A, bound(input.vx), bound(input.vy), bound(input.vz), input.vyaw);
        vehicle->control->flightCtrl(cd);
        cout << "real control signal is: " << cd.x << " " << cd.y << " " << cd.z << " " << cd.yaw << "\n";
    }
    else
    {
        cout << "gear need to pull" << endl;
    }
}

void communicator::controlTest()
{
    char c = scanKeyboard();
    float x = 0, y = 0, z = 0, yaw = 0;
    //Control::VERTICAL_VELOCITY;
    switch (c)
    {
    case 'w':
        z += 0.5;
        break;
    case 's':
        z -= 0.5;
        break;
    case 'a':
        y += 0.2;
        break;

    case 'd':
        y -= 0.2;
        break;
    default:
        break;
    }
    printf("control input is: %f %f %f %f", x, y, z, yaw);
    Control::CtrlData conInput(0x4A, x, y, z, yaw);
    vehicle->control->flightCtrl(conInput);
}

void communicator::readyCalback(const std_msgs::Int8 &ready)
{
    if (ready.data == 1)
    {
        int8_t tmp = 1;
        tmp = tmp << seq;
        synch |= tmp;
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

void communicator::loop(ros::Publisher &pub, ros::Publisher &pubDec)
{
    if (decFlag)
    {
        uav_tracking::packsDec tmp;
        tmp.pack.push_back(selfDec);
        for (auto i : othersDec)
        {
            tmp.pack.push_back(i.second);
        }
        tmp.yaw = yawAngle;
        tmp.synch = synch;
        pubDec.publish(tmp);
    }
    else
    {
        uav_tracking::packs tmp;
        tmp.pack.push_back(self);
        for (auto i : others)
        {
            tmp.pack.push_back(i.second);
        }
        tmp.yaw = yawAngle;
        tmp.synch = synch;
        pub.publish(tmp);
    }
}

void communicator::print()
{
    if (!decFlag)
    {
        dataSelf << self.x << "," << self.vx << "," << self.y << "," << self.vy << "," << self.yaw << endl;
        for (auto i : others)
        {
            dataXb << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
            cout << "xbee data is: " << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
        }
    }
    else
    {
        dataSelf << selfDec.x << "," << selfDec.vx << "," << selfDec.y << "," << selfDec.vy << "," << selfDec.yaw << endl;
        for (auto i : othersDec)
        {
            dataXb << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
            cout << "xbee data is: " << i.second.number << "," << i.second.x << "," << i.second.vx << "," << i.second.y << "," << i.second.vy << "," << i.second.yaw << endl;
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "com");
    ros::NodeHandle nh;
    communicator com(argv[1]);

    int calibration = 1;
    cout << "please input the cali mode: not 0 for calibration completed; 0, do calibration" << endl;
    cin >> calibration;
    com.cali(calibration);
    cout << "cali complete press any button to continue." << endl;
    getchar();

    ros::Publisher pub = nh.advertise<uav_tracking::packs>("posvel_msg", 2);
    ros::Publisher pubDec = nh.advertise<uav_tracking::packsDec>("posvel_msg_dec", 2);
    ros::Subscriber sub = nh.subscribe("controlData", 1, &communicator::controlCallback, &com);
    ros::Subscriber subGo = nh.subscribe("readyTogo", 2, &communicator::readyCalback, &com);
    ros::Subscriber subxy = nh.subscribe("xyDec", 1, &communicator::xyCallback, &com);
    ros::Rate loop_rate(15);
    int i = 0;
    /*com.initVehicle();
    Vehicle *v = com.getVehicle();
    v->control->takeoff(1);
    while (ros::ok())
    {
        loop_rate.sleep();
        com.controlTest();
        ros::spinOnce();
    }
    v->releaseCtrlAuthority(10);
    return -1;*/
    while (ros::ok())
    {
        loop_rate.sleep();
        if (i == 0)
        {
            com.getSelf();
            com.sendInfo();
            com.getInfoFromOthers();
            com.loop(pub, pubDec);
            com.print();
        }
        i++;
        i %= 3;
        ros::spinOnce();
    }
    return 0;
}