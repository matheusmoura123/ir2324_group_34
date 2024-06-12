#ifndef group34_pkg1_CILINDER_FINDER_H
#define group34_pkg1_CILINDER_FINDER_H

//for laser search
#include <iostream>
#include <stdio.h>   
#include <cmath>
#include <math.h>
#include <vector>

using namespace std;

#define PI 3.14159265

//laser characteristics
#define LASER_ANGLE_MAX 1.91986
#define LASER_ANGLE_MIN -1.91986
#define LASER_DELTA 0.005774 
#define LASER_COUNT 666
#define CILINDER_R 0.21
#define LASER_OFFSET_X 0.20

//initial pose of tiago
#define INITIAL_X -6.584825
#define INITIAL_Y 1.376042
#define INITIAL_Z -0.000740
#define INITIAL_ROLL 0.000234
#define INITIAL_PITCH 0.000284
#define INITIAL_YAW -0.000216


double dist(int k, double d0, double delta, double r);

vector<double> center_cart(double d0, double theta0, double r);

vector<double> center_polar(double d0, double theta0, double r);

vector<vector<double>> cilinder_wf(vector<vector<double>> cilinders, vector<double> tiago_pose);

vector<vector<double>> cilinder_scan(vector<double> laser, vector<double> tiago_pose, double delta, double r, double laser_angle_min);

#endif //group34_pkg1_CILINDER_FINDER_H
