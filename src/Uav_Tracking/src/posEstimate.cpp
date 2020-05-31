#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

#define ITERATION 20
#define NODE 2
#define ESTIMATEPARA 3 //pos x, pos y, yaw
#define ALPHA 0.5
#define TIMEINTERVAL 1

using namespace cv;
using namespace std;

/*
 *TODO: 1. time interval for velocity estimate. A parameter

*/

class posEstimateDec
{
private:
    /* data */
    Mat grad;

    Mat topo;
    Mat IW;
    Mat wtilder;

    Scalar lastxy;

    Scalar xy;
    Mat xy_1;
    Mat xy_2;

    int num;

public:
    posEstimateDec();
    posEstimateDec(int number, Mat &topology);

    Mat gradient(Mat states, Mat xys);

    void setTopoNum(int number, Mat &topology);
    Scalar position(Mat selfstate, Mat xys, int n);
    Scalar position() { return xy; }
    Scalar velocity()
    {
        Scalar res = (xy - lastxy) / TIMEINTERVAL;
        return Scalar(0, res[0], 0, res[2]);
    }
};
posEstimateDec::posEstimateDec()
{
    xy = {0., 0., 0., 0.};
}
//useless for now
posEstimateDec::posEstimateDec(int number, Mat &topology)
{
    xy = {0., 0., 0., 0.};
    num = number;
    topo = topology.clone();
    Mat I = Mat::eye(topology.size(), CV_64FC1);
    IW = I + topo;
    wtilder = IW / 2;
}
/*
@param number: the sequence number of this node
@param topology: the topology of the network 
*/
void posEstimateDec::setTopoNum(int number, Mat &topology)
{
    num = number;
    topo = topology.clone();
    topo.convertTo(topo, CV_64FC1);
    Mat I = Mat::eye(topology.size(), CV_64FC1);
    IW = I + topo;
    wtilder = IW / 2;
}
/*
@param states: the states of all the nodes
@param xys: the xy of all the nodes
*/
Mat posEstimateDec::gradient(Mat states, Mat xys)
{
    Mat A = states.col(2);
    Mat B = states.col(1) - states.col(0).mul(A);

    Mat result = IW.clone();
    result.col(0) = 2 * (A.mul(A).mul(xys.col(0)) + A.mul(B) - A.mul(xys.col(1))).mul(1 / (A.mul(A) + 1));
    result.col(1) = 2 * (xys.col(1) - A.mul(xys.col(0)) - B).mul(1 / (A.mul(A) + 1));
    return result;
}
//xy: (x,0,y,0)
/*
@param states: positions and angles from the network nodes
@param xys: computed xy of all the nodes
@param n: iteration times
*/
Scalar posEstimateDec::position(Mat states, Mat xys, int n)
{
    Mat gradNow = gradient(states, xys);
    if (n > 1)
    {
        lastxy = xy;

        xy_2 = xy_1.clone();
        xy_1 = xys.clone();

        xys = IW * xy_1 - wtilder * xy_2 - ALPHA * (gradNow - grad);
        grad = gradNow.clone();
    }
    else
    {
        xy_1 = xys.clone();
        xys = topo * xys - ALPHA * gradNow;
        grad = gradNow.clone();
    }
    xy[0] = xys.at<double>(num - 1, 0);
    xy[2] = xys.at<double>(num - 1, 1);
    return xy;
}

class posEstimate
{
private:
    Scalar xy;
    Scalar lastXy;

public:
    posEstimate() {}
    posEstimate(vector<vector<double>> initData, int series);
    void init(vector<vector<double>> initData, int series); //input the data of one moment, loop in the outer function
    Scalar gradient(Mat state, Scalar xy);
    Scalar position();
    Scalar velocity();
    Scalar position(Mat s);
};
//the third column of the input data
Scalar posEstimate::gradient(Mat state, Scalar xy)
{
    Mat A = state.col(2);
    Mat B = state.col(1) - state.col(0).mul(A);
    double data[2];
    data[0] = sum(2 * (A.mul(A) * xy[0] + A.mul(B) - A * xy[2]) / (A.mul(A) + 1))[0];
    data[1] = sum(2 * (xy[2] - A * xy[0] - B) / (A.mul(A) + 1))[0];
    return Scalar(data[0], 0, data[1], 0);
}
/*
*   initData format:
    every interval number of double byte is one set data of one moment
    inside of it is:
    the first column is pos x
    the second is pos y
    the third is tangent

    that is: one row is for the whole state of one node
*/

posEstimate::posEstimate(vector<vector<double>> initData, int series)
{
    init(initData, series);
}

void posEstimate::init(vector<vector<double>> initData, int series)
{
    //data interval
    int interval = NODE * ESTIMATEPARA;
    //output estimated xy
    xy = Scalar(2, 0);
    Scalar avg;
    Mat piece(NODE, ESTIMATEPARA, CV_64FC1);
    for (int i = 0; i < ITERATION; i++)
    {
        for (int j = 0; j < series; j++)
        {
            piece.data = (uchar *)(initData[j].data());
            avg += gradient(piece, xy);
        }
        avg /= series;
        xy = xy - ALPHA * avg;
    }
    lastXy = xy;
}

Scalar posEstimate::position(Mat s)
{
    for (int i = 0; i < ITERATION; i++)
    {
        xy = xy - ALPHA * gradient(s, xy);
    }
    return xy;
}

//time interval to be set. A parameter
Scalar posEstimate::velocity()
{
    Scalar res = (xy - lastXy) / TIMEINTERVAL;
    lastXy = xy;
    return Scalar(0, res[0], 0, res[2]);
}

Scalar posEstimate::position()
{
    return xy;
}
