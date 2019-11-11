#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

#define ITERATION 20
#define NODE 3
#define ESTIMATEPARA 3 //pos x, pos y, yaw
#define ALPHA 0.01
#define TIMEINTERVAL 0.05

using namespace cv;
using namespace std;

/*
 *TODO: 1. time interval for velocity estimate. A parameter

*/
class posEstimate
{
private:
    Mat grad;
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
