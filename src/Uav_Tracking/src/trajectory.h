#pragma once
#include <vector>
#include <cassert>
#include <queue>
#define QSIZE 10
//#define SWITCH
using namespace std;
class Trajectory
{
private:
	float y, z;
	vector<float> pos;
	queue<pair<float, float>> filter;
	pair<float, float> filterSum;

public:
	Trajectory()
	{
		for (int i = 0; i < QSIZE; i++)
		{
			filter.push({0, 0});
		}
		filterSum = {0, 0};
	}
	void init(vector<float> current_pose)
	{
		assert(current_pose.size() == 3);
		pos.resize(4);
		y = current_pose[1];
		z = current_pose[2];
	}

	void generate(vector<float> current_pose, vector<float> team)
	{
		assert(current_pose.size() == 3);
		pos[3] = (current_pose[0] * z) / current_pose[2]; //yaw
		team[0] = bound(team[0]);
		team[1] = bound(team[1]);
		/*if (team[0] > 1)
		{
			team[0] = pow(team[0], 0.5);
		}
		if (team[1] > 1)
		{
			team[1] = pow(team[1], 0.5);
		}*/
		pair<float, float> old = filter.back();
		filter.pop();
		filter.push({team[0], team[1]});
		filterSum.first += team[0] - old.first;
		filterSum.second += team[1] - old.second;
		pos[0] = filterSum.first / QSIZE;
		pos[1] = filterSum.second / QSIZE;

		//pos[2] = -(current_pose[1] - y) / current_pose[2]; //vertical
#ifdef SWITCH
		if (current_pose[2] < z * 0.8 || current_pose[2] > z * 1.3)
		{
			pos[0] = 1 - z / current_pose[2];
			pos[1] = (current_pose[0]) / current_pose[2];
			pos[2] = -(current_pose[1] - y) / current_pose[2];
			pos[3] = 0;
		}
		else
#endif
		{
#ifdef SWITCH
			pos[0] = 0;
#endif
#ifndef SWITCH
			//pos[0] = 1 - z / current_pose[2];
#endif
			//pos[1] = 0;
			//pos[2] = -(current_pose[1] - y) / current_pose[2];
			//pos[3] = (current_pose[0] * z) / current_pose[2];
		}
	}

	//old version
	/*void generate(vector<float> current_pose, vector<float> obs_input)
	{
		assert(current_pose.size() == 3);
#ifdef SWITCH
		if (current_pose[2] < z * 0.8 || current_pose[2] > z * 1.3)
		{
			pos[0] = 1 - z / current_pose[2];
			pos[1] = (current_pose[0]) / current_pose[2];
			pos[2] = -(current_pose[1] - y) / current_pose[2];
			pos[3] = 0;
		}
		else
#endif
		{
#ifdef SWITCH
			pos[0] = 0;
#endif
#ifndef SWITCH
			pos[0] = 1 - z / current_pose[2];
#endif
			pos[1] = 0 + obs_input[0];	//one part from tracking, another part is from obs avoidance
			cout << "pos[1] " << pos[1] << endl;
			pos[2] = -(current_pose[1] - y) / current_pose[2];
			pos[3] = (current_pose[0] * z) / current_pose[2];
		}
	}*/
	//add boundary
	float bound(const float &input)
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
	vector<float> position()
	{
		return pos;
	}
};
