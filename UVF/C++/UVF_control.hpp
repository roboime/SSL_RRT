#pragma once

#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <chrono>
#include "UVF.hpp"

int sign(double x);

tuple<double, double> control(tuple<double,double,double> point, tuple<double,double,double> target, vector<tuple<double, double>> obstacle_vector, 
            double de, double kr, double d_min, double delta);