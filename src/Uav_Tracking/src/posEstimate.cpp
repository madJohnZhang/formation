#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

#define ITERATION 20
#define NODE 2
#define ESTIMATEPARA 3 //pos x, pos y, yaw
#define ALPHA 0.01
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
    Scalar grad;

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

    Scalar gradient(Mat selfstate);

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

void posEstimateDec::setTopoNum(int number, Mat &topology)
{
    num = number;
    topo = topology.clone();
    topo.convertTo(topo, CV_64FC1);
    Mat I = Mat::eye(topology.size(), CV_64FC1);
    IW = I + topo;
    wtilder = IW / 2;
}

Scalar posEstimateDec::gradient(Mat selfstate) //self state only
{
    Mat A = selfstate.col(2);
    Mat B = selfstate.col(1) - selfstate.col(0).mul(A);

    double data[2];
    data[0] = sum(2 * (A.mul(A) * xy[0] + A.mul(B) - A * xy[2]) / (A.mul(A) + 1))[0];
    data[1] = sum(2 * (xy[2] - A * xy[0] - B) / (A.mul(A) + 1))[0];
    return Scalar(data[0], 0, data[1], 0);
}
//xy: (x,0,y,0)
Scalar posEstimateDec::position(Mat selfstate, Mat xys, int n)
{
    Scalar gradNow = gradient(selfstate);
    if (n > 1)
    {
        lastxy = xy;

        xy_2 = xy_1.clone();

        xys.row(num - 1) = Scalar(xy[0], xy[2]);
        xy_1 = xys.clone();

        Scalar gradMinus = gradNow - grad;
        Mat minus(1, 2, CV_64FC1);
        minus.at<double>(0) = gradMinus[0];
        minus.at<double>(1) = gradMinus[2];
        cout << "gradMinus" << gradMinus << endl;
        cout << "minus" << minus << endl;
        Mat tmp = IW.row(num - 1) * xy_1 - wtilder.row(num - 1) * xy_2 - ALPHA * minus;
        xy[0] += tmp.at<double>(0);
        xy[2] += tmp.at<double>(1);
        grad = gradNow;
    }
    else
    {
        xy_1 = xys.clone();
        xy = xy - ALPHA * gradNow;
        grad = gradNow;
    }
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
