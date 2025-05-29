//
// Created by Ebert on 5/28/2025.
//

#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <utility>
#include <cmath>

namespace navigation {
    struct ControllerInput {
        std::pair<double, double> current_position;
        std::pair<double, double> target_position;
        std::pair<double, double> current_velocity;
        double max_velocity;
        double acceleration;
        double current_orientation;
        double delta_time;
        double PID_Kp;
        double PID_Kd;
        double PID_Ki;
        double PID_Threshold;
        double error_integral;
        double previous_error;
    };
class BangBangController2D {
public:
    explicit BangBangController2D(const ControllerInput &input_data);
    std::pair<double, double> Compute2DVelocity();
    void UpdateState(const std::pair<double, double>& current_position, const std::pair<double, double>& current_velocity);
    static double Distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
    static double SignedDistance(const std::pair<double, double> &a, const std::pair<double, double> &b, double orientation);
private:
    std::pair<double,double> FormatOutputVelocity(std::pair<double,double> calculated_velocity);
    double Compute1DTime(double distance, double velocity, double max_velocity, double acceleration);
    double CalculateTimeDifference(double alpha);
    double FindDecompositionAngle();
    double Compute1DVelocity(double current_velocity, double current_position, double target_position, double acceleration, double max_velocity);
    std::pair<double,double> ComputePIDVelocity();
    std::pair<double, double> CurrentPosition;
    std::pair<double, double> TargetPosition;
    std::pair<double, double> CurrentVelocity;
    double MaxVelocity;
    double Acceleration;
    double CurrentOrientation;
    double DeltaTime;
    const double EPSILON = 1e-5;
    double PID_THRESHOLD;
    double PID_Kp;
    double PID_Kd;
    double PID_Ki;
    double PreviousError;
    double ErrorIntegral;
};

} // navigation

#endif //NAVIGATION_H
