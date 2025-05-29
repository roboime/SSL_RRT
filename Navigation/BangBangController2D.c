//
// Created by Ebert on 5/28/2025.
//
#include "pch.h"
#include "BangBangController2D.h"

#include <algorithm>
#include <math.h>
#define M_PI 3.14159265358979323846

namespace navigation {
    BangBangController2D::BangBangController2D(const ControllerInput &input_data) :
    CurrentPosition(input_data.current_position),
    TargetPosition(input_data.target_position),
    CurrentVelocity(input_data.current_velocity),
    MaxVelocity(input_data.max_velocity),
    Acceleration(input_data.acceleration),
    CurrentOrientation(input_data.current_orientation),
    DeltaTime(input_data.delta_time),
    PID_Kp(input_data.PID_Kp),
    PID_Kd(input_data.PID_Kd),
    PID_Ki(input_data.PID_Ki),
    PID_THRESHOLD(input_data.PID_Threshold),
    PreviousError(input_data.previous_error),
    ErrorIntegral(input_data.error_integral)
    {}
    double BangBangController2D::Compute1DTime(double distance, double current_velocity, double max_velocity, double acceleration) {
        current_velocity = std::abs(current_velocity);
        distance = std::abs(distance);
        double break_time = current_velocity / acceleration;
        double break_distance = current_velocity * break_time / 2.0;
        double remaining_distance = distance - break_distance;
        double required_max_velocity = std::sqrt(2*acceleration*remaining_distance + current_velocity*current_velocity);
        if (required_max_velocity < max_velocity) {
            double acceleration_time = (required_max_velocity - current_velocity) / acceleration;
            return acceleration_time + break_time;
        }
        double acceleration_time = (max_velocity - current_velocity) / acceleration;
        double acceleration_distance = (max_velocity + current_velocity) * acceleration_time / 2.0;
        double constant_distance = distance - break_distance - acceleration_distance;
        double constant_time = constant_distance / max_velocity;
        return acceleration_time + constant_time + break_time;
    }
    double BangBangController2D::CalculateTimeDifference(double alpha) {
        double max_velocity_x = MaxVelocity * std::cos(alpha);
        double max_acceleration_x = Acceleration * std::cos(alpha);
        double max_velocity_y = MaxVelocity * std::sin(alpha);
        double max_acceleration_y = Acceleration * std::sin(alpha);
        double distance_x = std::abs(TargetPosition.first - CurrentPosition.first);
        double distance_y = std::abs(TargetPosition.second - CurrentPosition.second);
        double time_x = Compute1DTime(distance_x, max_velocity_x, max_acceleration_x, alpha);
        double time_y = Compute1DTime(distance_y, max_velocity_y, max_acceleration_y, alpha);
        return time_x - time_y;
    }
    double BangBangController2D::FindDecompositionAngle() {
        double alpha_low = 0.0;
        double alpha_high = M_PI / 2;
        double optimal_alpha = M_PI / 4;
        while (alpha_high - alpha_low > EPSILON) {
            double alpha = (alpha_low + alpha_high) / 2.0;
            double delta_time = CalculateTimeDifference(alpha);
            if (std::abs(delta_time) < EPSILON) {
                break;
            }
            if (delta_time > 0.0) {
                alpha_high = alpha;
            } else {
                alpha_low = alpha;
            }
        }
        optimal_alpha = (alpha_low + alpha_high) / 2.0;
        return optimal_alpha;
    }
    double BangBangController2D::Compute1DVelocity(double current_velocity, double current_position, double target_position, double acceleration, double max_velocity) {
        double distance = std::abs(target_position - current_position);
        double sign = (target_position - current_position) >= 0 ? 1.0 : -1.0;
        double breakDistance = current_velocity * current_velocity / (2.0 * acceleration);
        double delta = 0.0;
        if (distance <= breakDistance) {
            delta -= acceleration * DeltaTime;
        } else {
            delta += acceleration * DeltaTime;
        }
        double new_velocity = current_velocity + delta * sign;
        new_velocity = std::clamp(new_velocity, -max_velocity, max_velocity);
        return new_velocity;
    }
    std::pair<double, double> BangBangController2D::FormatOutputVelocity(std::pair<double,double> calculated_velocity) {
        auto [velocity_x, velocity_y] = calculated_velocity;
        double velocity_tangent =  velocity_x * cos(-CurrentOrientation) + velocity_y * sin(-CurrentOrientation);
        double velocity_normal  = -velocity_x * sin(-CurrentOrientation) + velocity_y * cos(-CurrentOrientation);
        return std::make_pair(velocity_tangent, velocity_normal);
    }
    std::pair<double, double> BangBangController2D::Compute2DVelocity() {
        if (Distance(CurrentPosition, TargetPosition) < PID_THRESHOLD) {
            return ComputePIDVelocity();
        }
        double alpha = FindDecompositionAngle();
        double max_velocity_x = MaxVelocity * std::cos(alpha);
        double max_velocity_y = MaxVelocity * std::sin(alpha);
        double acceleration_x = Acceleration * std::cos(alpha);
        double acceleration_y = Acceleration * std::sin(alpha);
        double new_velocity_x = Compute1DVelocity(CurrentVelocity.first,CurrentPosition.first,
            TargetPosition.first, acceleration_x, max_velocity_x);
        double new_velocity_y = Compute1DVelocity(CurrentVelocity.second, CurrentPosition.second,
            TargetPosition.second, acceleration_y, max_velocity_y);
        std::pair<double,double> calculated_velocity = std::make_pair(new_velocity_x, new_velocity_y);
        //std::pair<double,double> output_velocity = FormatOutputVelocity(calculated_velocity);
        return calculated_velocity;
    }
    std::pair<double,double> BangBangController2D::ComputePIDVelocity() {
        double error = SignedDistance(CurrentPosition, TargetPosition, CurrentOrientation);
        double integral = ErrorIntegral + error * DeltaTime;
        double derivative = (error - PreviousError) / DeltaTime;
        double output = PID_Kp * error + PID_Ki * integral + PID_Kd * derivative;

        double orientation_x = std::cos(CurrentOrientation);
        double orientation_y = std::sin(CurrentOrientation);
        double velocity_tangent = output * orientation_x;
        double velocity_normal = output * orientation_y;
        return std::make_pair(velocity_tangent, velocity_normal);
    }
    void BangBangController2D::UpdateState(const std::pair<double, double>& current_position, const std::pair<double, double>& current_velocity) {
        CurrentPosition = current_position;
        CurrentVelocity = current_velocity;
        double error = SignedDistance(CurrentPosition, TargetPosition, CurrentOrientation);
        PreviousError = error;
        ErrorIntegral += error * DeltaTime;
    }
    double BangBangController2D::Distance(const std::pair<double, double> &a, const std::pair<double, double> &b) {
        double dx = a.first - b.first;
        double dy = a.second - b.second;
        return std::sqrt(dx * dx + dy * dy);
    }
    double BangBangController2D::SignedDistance(const std::pair<double, double> &a, const std::pair<double, double> &b, double orientation){
        double dx = b.first - a.first;
        double dy = b.second - a.second;
        double orientation_x = std::cos(orientation);
        double orientation_y = std::sin(orientation);
        double dot = dx * orientation_x + dy * orientation_y;
        double sign = dot >= 0 ? 1.0 : -1.0;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance * sign;
    }
} // navigation