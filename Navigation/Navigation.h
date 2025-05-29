#ifndef RRT_H
#define RRT_H

#define _CRTDBG_MAP_ALLOC

// remove stupid MSVC min/max macro definitions
#undef min
#undef max
//#define _DEBUG
#ifdef _DEBUG
#define NAVIGATION_API
#define __cdecl
#else
#define NAVIGATION_API extern "C"

#endif

NAVIGATION_API __declspec(dllexport) int navigation_api(double current_position_x, double current_position_y,
    double target_position_x, double target_position_y,
    double current_velocity_x, double current_velocity_y,
    double max_velocity,
    double acceleration,
    double current_orientation,
    double* output_velocity_x, double* output_velocity_y,
    double delta_time,
    double pid_kp, double pid_kd, double pid_ki, double pid_threshold,
    double error_integral, double previous_error);

#endif