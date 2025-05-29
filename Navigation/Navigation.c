// dllmain.cpp : Defines the entry point for the DLL application.
//Using VS2019

#include "pch.h"
#include "Navigation.h"
#include "BangBangController2D.h"

__declspec(dllexport) int navigation_api(double current_position_x, double current_position_y,
    double target_position_x, double target_position_y,
    double current_velocity_x, double current_velocity_y,
    double max_velocity,
    double acceleration,
    double current_orientation,
    double* output_velocity_x, double* output_velocity_y,
    double delta_time,
    double pid_kp, double pid_kd, double pid_ki, double pid_threshold,
    double error_integral, double previous_error) {
    std::pair<double, double> current_position = std::make_pair(current_position_x, current_position_y);
    std::pair<double, double> target_position = std::make_pair(target_position_x, target_position_y);
    std::pair<double, double> current_velocity = std::make_pair(round(current_velocity_x * 1000) / 1000, round(current_velocity_y * 1000) / 1000);
    navigation::ControllerInput controllerInput = {
        current_position, target_position, current_velocity,
        max_velocity, acceleration, current_orientation, delta_time, pid_kp, pid_kd, pid_ki, pid_threshold, error_integral, previous_error };
    navigation::BangBangController2D controller(controllerInput);
    std::pair<double, double> output_velocity = controller.Compute2DVelocity();
    *output_velocity_x = round(output_velocity.first * 1000) / 1000;
    *output_velocity_y = round(output_velocity.second * 1000) / 1000;
    return 0;
}

