#ifndef RRT_H
#define RRT_H

#include "Environment.h"
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <crtdbg.h>


#define _CRTDBG_MAP_ALLOC


// remove stupid MSVC min/max macro definitions
#undef min
#undef max

#ifdef _DEBUG
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
#define DBG_NEW new
#endif

#define RRT_API extern "C" __declspec(dllexport)

using namespace std;

struct TesteStruct {

};

RRT_API int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug);
RRT_API int teste(int a);

bool CheckInputError(int* seed, int* pathsize, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug);

#endif