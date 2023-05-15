#ifndef RRT_H
#define RRT_H

#include "Environment.h"
#include <chrono>
#include <ctime>
#include <cstdlib>
//#include <crtdbg.h>


#define _CRTDBG_MAP_ALLOC


// remove stupid MSVC min/max macro definitions
#undef min
#undef max
//#define _DEBUG
#ifdef _DEBUG
#define RRT_API
#define __cdecl
#else
#define RRT_API extern "C" __declspec(dllexport)

#endif


using namespace std;

struct TesteStruct {

};

RRT_API int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug);
RRT_API int teste(int a);

bool is_valid(float x);
bool is_into_bounds(float x, float bound_lenght);

int CheckInputError(int* seed, int* pathsize, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug, float field_length, float field_width);

#endif