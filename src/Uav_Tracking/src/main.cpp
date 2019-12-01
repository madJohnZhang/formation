#include <dji_sdk/dji_sdk.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <queue>
#include <thread>
#include <cassert>
//#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <serial/serial.h>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <sys/time.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <djiosdk/dji_broadcast.hpp>

#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h>   //velocity
#include <sensor_msgs/LaserScan.h>          //obstacle distance & ultrasonic
#include <sensor_msgs/CameraInfo.h>         // camera info message. Contains cam params
#include "yaml-cpp/yaml.h"                  // use to parse YAML calibration file
#include <fstream>                          // required to parse YAML

//#include"inc2/trackOnUav.h"
#include "inc/tracker.cpp"
#include "detector.h"
#include "PID.h"
#include "Kalman.h"
#include "trajectory.h"
#include "obs.hpp"
#include "uav_tracking/posvel.h"
#include "uav_tracking/packs.h"
#include "uav_tracking/controldata.h"
#include "formation.cpp"

#define CONTROL

using namespace std;
using namespace ros;
using namespace cv;
using namespace DJI::OSDK;

mutex mtxCam;
mutex mtxPos;
Mat frame;

void getFrame(VideoCapture *cap)
{
    while (true)
    {
        mtxCam.lock();
        *cap >> frame;
        mtxCam.unlock();
    }
}

long long getSystemTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}
class transmission
{
    /* data */
public:
    int number;
    float x;
    float vx;
    float y;
    float vy;
    float yaw;
    transmission()
    {
        number = 2;
        x = 2.2;
        vx = 0.22;
        y = 2.2;
        vy = 0.22;
        yaw = 2.2;
    }
    transmission &operator+(int i)
    {
        x += i;
        vx += i;
        y += i;
        vy += i;
        yaw += i;
    }
};

void testXBEE()
{
    fstream dataSelf("dataSelftest.csv", ios::trunc | ios::out);
    fstream dataXb1("dataXb1test.csv", ios::trunc | ios::out);
    fstream dataXb2("dataXb2test.csv", ios::trunc | ios::out);

    serial::Serial ser;
    string port("/dev/ttyUSB1");
    int baudrate = 9600;
    serial::Timeout timeout(100);

    cout << "and here" << endl;
    ser.setTimeout(timeout);
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    ser.setParity(serial::parity_even);
    ser.open();
    ser.flushInput();
    ser.flushOutput();
    transmission send, recieve;
    uint8_t *din, *dout;
    din = new uint8_t[sizeof(transmission) * 2];
    dout = new uint8_t[sizeof(transmission)];
    ros::Rate loop(5);
    int i = 3;
    while (ros::ok())
    {
        //send data
        loop.sleep();
        ros::spinOnce();
        send + i;
        memcpy(dout, &send, sizeof(transmission));
        ser.write(dout, sizeof(transmission));
        size_t bites = ser.read(din, sizeof(transmission) * 2);
        for (int j = 0; j < 2; j++)
        {
            memcpy(&recieve, din + j * sizeof(transmission), sizeof(transmission));

            dataXb1 << recieve.number << "," << recieve.x << "," << recieve.vx << "," << recieve.y << "," << recieve.vy << "," << recieve.yaw << "," << bites << endl;
        }
    }
    dataXb1.close();
    dataXb2.close();
    exit(0);
}

int main(int argc, char **argv)
{

    /*FileStorage fs("formation.xml",FileStorage::WRITE);
    fs<<"formationNum"<<3;
    fs<<"seq"<<1;
    string port("/dev/ttyUSB1");
    fs<<"port"<<port;
    fs<<"baudrate"<<9600;
    Mat topo = (Mat_<int>(3,3)<< 1,1,1,0,1,0,1,0,0,0,0,1);
    Mat coefficients = (Mat_<float>(2,4)<<1,1,0,0,0,0,1,1);
    fs<<"topology"<<topo;
    fs<<"coefficients"<<coefficients;
    Mat f = (Mat_<float>(4,2) << -2.5, 2.5, 0, 0, 0, 0, 0, 0);
    fs<<"formation"<<f;
    fs.release();*/

    init(argc, argv, "main");
    NodeHandle nh;

    Formation formation(1, argv);
    ros::Subscriber sub = nh.subscribe("posvel_msg", 10, &Formation::packCallback, &formation);
    ros::Publisher pub = nh.advertise<uav_tracking::controldata>("controlData", 1);
    ros::Publisher ready = nh.advertise<std_msgs::Int8>("readyTogo", 2);
    Detector target_finder(argc, argv);
    StapleTracker visual_tracker;
    Kalman motion_estimator;
    Trajectory traj_generator;
    PID controller;
    //LEDcontroller led;
    const float con_gain = 0.3; //the impact of the control signals on the EKF
    queue<float> q_vx, q_vy, q_vz, q_vyaw;
    fstream log_file("log_date.log", ios::trunc | ios::out);
    fstream log_pos("log_pos.csv", ios::trunc | ios::out);
    clock_t timer, timer_fps;
    long long timer_ts;
    char name[64];
    timer = clock();
    int iterate_times = 0;
    bool target_lost = false;

    vector<int> bounding_box(4);
    Mat image;
    VideoCapture cap;
    //Mat frame;
    cap.open(0);
    if (!cap.isOpened())
    {
        log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " cannot open camera!" << endl;
        return 0;
    }
    else
    {
        log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " camera opened" << endl;
    }
    //thread thread_video(getFrame, &cap);

    //ser.open();
    //string read;
    /*while(1)
    {
	ser.read(read, 4);
	cout<<read<<endl;
	sleep(1000);
    }*/
    //first frame init
    waitKey(10);
    cap >> frame;
    frame.copyTo(image);
    while (!target_finder.find(image))
    {
        cap >> frame;
        frame.copyTo(image);
    }
    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " target found" << endl;
    //led.on();
    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " start tracking init" << endl;
    //init tracker
    visual_tracker.init(image, target_finder.bounding_box());
    //init EKF
    motion_estimator.init(visual_tracker.position(), true);
    //init trajectory
    traj_generator.init(motion_estimator.position());

    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " init tracker" << endl;
    log_pos << visual_tracker.position()[0] << "," << visual_tracker.position()[1] << "," << visual_tracker.position()[2] << ",";

    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " init kalman" << endl;
    log_pos << motion_estimator.position()[0] << "," << motion_estimator.position()[1] << "," << motion_estimator.position()[2] << ",";

    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " init traj_gen" << endl;
    log_pos << traj_generator.position()[0] << "," << traj_generator.position()[1] << "," << traj_generator.position()[2] << "," << traj_generator.position()[2] << endl;

#ifdef CONTROL
    //init PID
    controller.init("para.txt");
    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " start main tracking loop" << endl;

    //init Guidance
    //initObs(10, 0.03);
#endif
    //initial counter
    int posInitCounter = 0;
    timer_ts = getSystemTime();
    timer_fps = clock();
    ros::Rate loop_rate(20);
    int cooldown = 0; //cooldown for the yaw adjustment

    std_msgs::Int8 goFly;
    goFly.data = 1;
    ready.publish(goFly);
    ready.publish(goFly);
    //main loop
    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();
        cap >> frame;
        cout << "here1" << endl;
        frame.copyTo(image);
        cout << "copy complete" << endl;
        try
        {
            target_lost = !visual_tracker.track(image);
        }
        catch (const std::exception &e)
        {
            std::cerr << "track dump" << '\n';
        }

        if (target_lost)
        {
            log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " target lost!" << endl;
        }
        /*
	if(!visual_tracker.track(image)){
            //led.off();
            log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " target lost" << endl;
            while(!target_finder.find(image)){
                frame.copyTo(image);
                drone->attitude_control(0x48,0,0,0,0);
            }
            log_file<<(clock() - timer)/(float)CLOCKS_PER_SEC<<" target found"<<endl;
            //led.on();
            visual_tracker.init(image,target_finder.bounding_box());
            motion_estimator.init(visual_tracker.position(),true);
        }
*/
        cout << "here2" << endl;
        if (!(iterate_times % 8) /*&&!target_lost*/) //templates update frequency
        {
            visual_tracker.update(image);
        }
        visual_tracker.drawCurrentImg(image);
        //imshow("Tracking API", image);
        log_pos << visual_tracker.position()[0] << "," << visual_tracker.position()[1] << "," << visual_tracker.position()[2] << ",";

//montion estimator
#ifdef CONTROL
        if (q_vx.size() > 7)
        { //size of control signal buffer
            motion_estimator.update(visual_tracker.position(), q_vx.front() * con_gain, q_vy.front() * con_gain, q_vz.front() * con_gain, q_vyaw.front() * 3.14159 / 180.0 * con_gain, getSystemTime() - timer_ts);
            q_vx.pop();
            q_vy.pop();
            q_vz.pop();
            q_vyaw.pop();
        }
        else
#endif
        {
            motion_estimator.update(visual_tracker.position(), 0, 0, 0, 0, getSystemTime() - timer_ts);
        }
        log_pos << ","
                << "motion estimate"
                << "," << motion_estimator.position()[0] << "," << motion_estimator.position()[1] << "," << motion_estimator.position()[2] << ",";
        timer_ts = getSystemTime();
        cout << "here3" << endl;

        vector<float> formationInput(2, 0);
        if (Formation::count <= SERIES)
        {
            formation.init();
        }
        else
        {
            Mat tmp = formation.getInput();
            formationInput[0] = tmp.at<float>(0);
            formationInput[1] = tmp.at<float>(1);
            cout << "formationinput" << formationInput[0] << " " << formationInput[1] << endl;
        }
        //formation.print(&dataSelf, &dataXb);
        //trajectory generator
        //obstacle input removed temporarily
        //vector<float> obs_input = getAvoidanceInput();
        traj_generator.generate(motion_estimator.position(), formationInput);
        log_pos << ","
                << "traj_generator"
                << "," << traj_generator.position()[0] << "," << traj_generator.position()[1] << "," << traj_generator.position()[2] << "," << traj_generator.position()[3];

#ifdef CONTROL
        //PID controller and UAV communicator
        Control::CtrlData conInput(Control::STABLE_ENABLE | Control::HORIZONTAL_BODY | Control::HORIZONTAL_VELOCITY | Control::VERTICAL_POSITION, 0, 0, 2, 0);

        controller.update(traj_generator.position());
        log_pos << ","
                << "control signal"
                << "," << controller.vx << "," << controller.vy << "," << controller.vz << "," << controller.vyaw << endl;
        q_vx.push(controller.vx);
        q_vy.push(controller.vy);
        q_vz.push(controller.vz);
        q_vyaw.push(controller.vyaw);
        cout << "obs input is: " << controller.vy << endl;
        conInput = Control::CtrlData(0x4A, controller.vx, controller.vy, controller.vz, controller.vyaw); //controller.vz
        uav_tracking::controldata cInput;
        cInput.vx = conInput.x;
        cInput.vy = conInput.y;
        cInput.vz = conInput.z;
        cInput.vyaw = conInput.yaw;
        pub.publish(cInput);
#endif
#ifndef CONTROL
        log_pos << endl;
#endif
        if (!(iterate_times % 50))
        { //image output rate
            vector<int> param = vector<int>(2);
            sprintf(name, "/home/sustec/tracking/images/%03d.jpg", iterate_times);
            param[0] = CV_IMWRITE_JPEG_QUALITY;
            param[1] = 95;
            imwrite(name, image, param);
        }
        //write log before & after every step
        iterate_times++;
        log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " All right, " << (float)(CLOCKS_PER_SEC * iterate_times) / (clock() - timer_fps) << "fps" << endl;
        //waitKey();
    }
    //main loop end

    /* release data transfer */
    int err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);
    log_file << (clock() - timer) / (float)CLOCKS_PER_SEC << " end tracking" << endl;
    return 0;
}
