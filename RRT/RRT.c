// dllmain.cpp : Defines the entry point for the DLL application.
//Using VS2019

#include "pch.h"
#include "RRT.h"

RRT_API int teste(int a) {
    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    while (1) {
        if (elapsed.count() > 1) break;
        finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
    }
    return a;
}

bool CheckInputError(int* seed, int* pathsize, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug) {
    {
        if (seed == nullptr or
            pathsize == nullptr or
            startAndEnd == nullptr or
            obsList == nullptr or
            lastTree == nullptr or
            newTree == nullptr or
            useSimplePath == nullptr or
            debug == nullptr) {
            return true;
        }

        volatile int* vseed = static_cast<volatile int*>(seed);
        volatile int* vpathSize= static_cast<volatile int*>(pathsize);
        volatile float* vstartAndEnd = static_cast<volatile float*>(startAndEnd);
        volatile float* vobsList = static_cast<volatile float*>(obsList);
        volatile float* vlastTree = static_cast<volatile float*>(lastTree);
        volatile float* vnewTree = static_cast<volatile float*>(newTree);
        volatile int* vuseSimplePath= static_cast<volatile int*>(useSimplePath);
        volatile float* vdebug = static_cast<volatile float*>(debug);
        try {
            *vseed = *vseed;
            *vpathSize = *vpathSize;
            for (int i = 0; i < 4; ++i) {
                vstartAndEnd[i] = vstartAndEnd[i];
            }
            for (int i = 0; i < obsLen; ++i) {
                vobsList[i] = vobsList[i];
            }
            for (int i = 0; i < lastTreeLen; ++i) {
                vlastTree[i] = vlastTree[i];
            }
            for (int i = 0; i < 1010; ++i) {
                vnewTree[i] = vnewTree[i];
            }
            *vuseSimplePath= *vuseSimplePath;
            *vdebug = *vdebug;
            return false;
        }
        catch (...) {
            return true;
        }
    }
}

RRT_API int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug) {

    if (CheckInputError(seed, pathsize, startAndEnd, obsList, obsLen, lastTree,
        lastTreeLen, newTree, useSimplePath, debug)) {
        return -1;
    }


    //INICIAR SEED
    srand(*seed);

    auto st = std::chrono::high_resolution_clock::now();

    //Setar variaveis
    //Criar Environment
    Environment env(field_length, field_width);
    //Start/end
    Node start(*(startAndEnd), *(startAndEnd + 1), nullptr);
    Node end(*(startAndEnd + 2), *(startAndEnd + 3), nullptr);
    //ObstacleList
    vector<pair<float, Node>> obsV;
    for (int i = 0; i < obsLen; i++) {
        obsV.push_back(make_pair(*(obsList + 3*i), Node(*(obsList + 3*i + 1), *(obsList +3*i +2), nullptr)));
    }
    ObstacleGrid obs(obsV);
    //lastTree
    vector<Node> lst {Node(0,0,nullptr)};
  
    for (int i = 0; i < lastTreeLen; i++) {
        lst.push_back(Node(*(lastTree + 2*i), *(lastTree + 2*i + 1), nullptr));
    }
    
    //Generate Tree
     Tree tr(start, end, probGoal, probWaypoint, env, obs, lst);
    //se nao for possivel calcular, usar SimplePath
      *useSimplePath = !tr.grow(step, threshold);
    if (*useSimplePath) {
        obsV.clear();
        lst.clear();
        *pathsize = 0;
        *debug = 0;
        return 0; 
    }
    else {
        
        vector<Node> path = tr.backtrack();
        int pathLen = path.size();

        for (int i = 0; i < pathLen; i++) {
           
            *(newTree + 2*i) = path[pathLen -1 - i]._x;
            *(newTree + 2*i + 1) = path[pathLen - 1 - i]._y;
        }
        obsV.clear();
        lst.clear();
        path.clear();
        *pathsize = pathLen;
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - st;
        double fps = 1 / elapsed.count();
        *debug = fps;
        return 0;
    }
}
