//
// Created by luffy7n on 18-11-10.
//

#ifndef DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
#define DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "framework.h"

using namespace std;

//typedef std::uint64_t hash_t;
//constexpr hash_t prime = 0x100000001B3ull;
//constexpr hash_t basis = 0xCBF29CE484222325ull;

typedef struct Input_Num
{
    float x;
    float y;
}Input_Num;

typedef struct POSINFO
{
    string position;
    float offset;
}POSINFO;


//hash_t hash_(char const* str);
//constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis);
POSINFO Plane_Straight_line_LR(const Input_Num &num1, const Input_Num &num2, Input_Num Rec_values);

POSINFO Plane_Straight_line_UD(bool direction, Input_Num &num1, const Input_Num &num2, Input_Num Rec_values);
float minkowsky(const vector<float > &v1, const vector<float> &v2, float m);
float calculateSimilarity(const vector<float> &v1, const vector<float> &v2);
float calculateVelocity(vector<float> &p);
float calculateLength(vector<float> &length);
void controlKey(int frame);

#endif //DOCKING_GUIDANCE2_TOOLS_FUNCTION_H