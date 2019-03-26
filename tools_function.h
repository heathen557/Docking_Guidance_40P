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

POSINFO Plane_Straight_line_LR(const Input_Num &num1,const Input_Num &num2,Input_Num Rec_values);
POSINFO Plane_Straight_line_UD(const Input_Num &num1,const Input_Num &num2,Input_Num Rec_values);
float minkowsky(const vector<float > &v1, const vector<float> &v2, float m);
float calculateSimilarity(const vector<float> &v1, const vector<float> &v2);
float calculateVelocity(vector<float> &p);
float calculateLength(vector<float> &length);
void controlKey(int frame);

#endif //DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
