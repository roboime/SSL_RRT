#pragma once

#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <chrono>

using namespace std;

double gauss(double r, double delta);

double HS(tuple<double,double,double> point , double tx, double ty, double de, double kr, double ccw);

double TUF(tuple<double,double,double> point, tuple<double,double,double> target, double de, double kr);

tuple<double, tuple<double,double> ,double> AUF(tuple<double,double,double> point, vector<tuple<double, double>> obstacle_vector);

double UVF(tuple<double,double,double> point, tuple<double,double,double> target, vector<tuple<double, double>> obstacle_vector, 
            double de, double kr, double d_min, double delta);
