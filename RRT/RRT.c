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

bool is_valid(float x) {
    return !isnan(x) && !isinf(x);
}

bool is_into_bounds(float x, float bound_lenght) {
    return (x >= -bound_lenght / 2) && (x <= bound_lenght / 2);

}

int CheckInputError(int* seed, int* pathsize, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug, float field_length, float field_width) {
    {
        if (!is_valid(field_length) || !is_valid(field_width)) return 10;
        if (seed == nullptr or
            pathsize == nullptr or
            startAndEnd == nullptr or
            obsList == nullptr or
            lastTree == nullptr or
            newTree == nullptr or
            useSimplePath == nullptr or
            debug == nullptr) {
            return 1;
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
        }
        catch (...) {
            return 2;
        }
        try{
            *vpathSize = *vpathSize;
        }
        catch (...) {
            return 3;
        }
        try {
            for (int i = 0; i < 2; ++i) {
                vstartAndEnd[2 * i] = vstartAndEnd[2 * i];
                vstartAndEnd[2 * i + 1] = vstartAndEnd[2 * i + 1];
                if (!is_valid(vstartAndEnd[2 * i]) || !is_into_bounds(vstartAndEnd[2 * i], field_length)) return 11;
                if (!is_valid(vstartAndEnd[2 * i+1]) || !is_into_bounds(vstartAndEnd[2 * i+1], field_width)) return 12;
            }
        }
        catch (...) {
            return 4;
        }
        try{
            for (int i = 0; i < obsLen; ++i) {
                vobsList[3*i] = vobsList[3*i];
                vobsList[3*i+1] = vobsList[3*i+1];
                vobsList[3*i+2] = vobsList[3*i+2];
                if (!is_valid(vobsList[3 * i])) return 13;
                if (!is_valid(vobsList[3 * i + 1]) || !is_into_bounds(vobsList[3 * i + 1], field_length)) return 14;
                if (!is_valid(vobsList[3 * i + 2]) || !is_into_bounds(vobsList[3 * i + 2], field_width)) return 15;
            }
        }
        catch (...) {
            return 5;
        }
        try {
            for (int i = 0; i < lastTreeLen; ++i) {
                vlastTree[2 * i] = vlastTree[2 * i];
                vlastTree[2 * i + 1] = vlastTree[2 * i + 1];
                if (!is_valid(vlastTree[2 * i]) || !is_into_bounds(vlastTree[2 * i], field_length)) return 16;
                if (!is_valid(vlastTree[2 * i + 1]) || !is_into_bounds(vlastTree[2 * i + 1], field_width)) return 17;
            }
        }
        catch (...) {
            return 6;
        }
        try {
            for (int i = 0; i < 1010; ++i) {
                vnewTree[2 * i] = vnewTree[2 * i];
                vnewTree[2 * i + 1] = vnewTree[2 * i + 1];
            }
        }
        catch (...) {
            return 7;
        }
        try {
            *vuseSimplePath = *vuseSimplePath;
        }
        catch (...) {
            return 8;
        }
        try {
            *vdebug = *vdebug;
        }
        catch (...) {
            return 9;
        }
        return 0;
    }
}

RRT_API int __cdecl SetEnv(int* seed, int* pathsize, float probGoal, float probWaypoint, float step, float threshold,
    float field_length, float field_width, float* startAndEnd, float* obsList, int obsLen, float* lastTree,
    int lastTreeLen, float* newTree, int* useSimplePath, float* debug) {

    int error = CheckInputError(seed, pathsize, startAndEnd, obsList, obsLen, lastTree,
        lastTreeLen, newTree, useSimplePath, debug, field_length, field_width);
    if (error) {
        return error;
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
    //vector<pair<float, Node>> obsV;     antigo
    vector<pair<string, vector<float>>> obsV;

    for (int i = 0; i < obsLen; i++) {

        /*        OBS: REVISAR FORMATO E TIPO DOS DADOS
        *(obsList + 3*i) -> string
        *(obsList + 3*i + 1) -> 



        */
        
        if (*(obsList + 3 * i) == "circle") {

        }

        obsV.push_back(make_pair(*(obsList + 3*i),      ));
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
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - st;
    double fps = 1 / elapsed.count();
    *debug = fps;
    return 0;
}
