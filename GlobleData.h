//
// Created by heathen on 18-10-11.
//



#include <X11/Intrinsic.h>

#include <iostream>
#include <zconf.h>
//#include "logger.h"

#ifndef DOCKING_GUIDANCE_GLOBLEDATA_H
#define DOCKING_GUIDANCE_GLOBLEDATA_H

//#define LOGID_I "info"
//#define LOGID_C "custom"
//#define LOGID_B "bin"



//void  init_log();


/*
 * 消息控制信息
 */
struct control_msg {
    const char *airplane_model;
    String ip;                 //不使用 预留
    int port;                  //不使用 预留
    float clip_min_height;
    float clip_max_height;
    float clip_left_position;
    float clip_right_position;
    float cluster_tolerance;
    int cluster_size_min;
    int cluster_size_max;
    float mild_point0_x;
    float mild_point0_y;
    float mild_point1_x;
    float mild_point1_y;
    float end_point0_x;
    float end_point0_y;
    float end_point1_x;
    float end_point1_y;

    float left_point0_x;
    float left_point0_y;
    float right_point0_x;
    float right_point0_y;
    float left_point1_x;
    float left_point1_y;
    float right_point1_x;
    float right_point1_y;
    float heigth_distance_threahold;

    /***********检测区域参数*********/
    int detectionModel;       //检测模式：0：自检  1：运行过程中检测
    int detecPointsSize;      //检测点个数
    float detecPointX[50];    //检测点的坐标
    float detecPointY[50];

    float safePointsSize;
    float safePointX[50];
    float safePointY[50];

    /******机型参数表中的参数********************/
//    float standard_aircraftLength;
//    float standard_wingLength;
//    float standard_engineSpace;

    float wingLength1;
    float wingLength2;
    float wingLengthAdjust;
    float wingLengthPosOffset;
    float wingLengthNegOffset;
    float engineInner;
    float engineInnerAdjust;
    float engineInnerPosOffset;
    float engineInnerNegOffset;
    float engineOuter;
    float engineOuterAdjust;
    float engineOuterPosOffset;
    float engineOuterNegOffset;
    float planeLength;
    float planeLengthAdjust;
    float planeLengthPosOffet;
    float planeLengthNegOffset;
    float planeHeight;
    float planeHeightPosOffset;
    float planeHeightNegOffset;
    float planeWidth;
    float planeWidthPosOffset;
    float planeWidthNegOffset;
    float noseHeight;
    float noseHeightPosOffset;
    float noseHeightNegOffset;
    float noseDoor;
    float noseDoorPosOffset;
    float noseDoorNegOffset;
} ;

//消息队列的声明
struct my_msg_st {
    long int my_msg_type;
    char some_text[4096];
};







#endif //DOCKING_GUIDANCE_GLOBLEDATA_H
