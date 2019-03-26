//
// Created by heathen on 18-10-11.
//



#include <X11/Intrinsic.h>

#include <iostream>
#include <zconf.h>
#include "logger.h"

#ifndef DOCKING_GUIDANCE_GLOBLEDATA_H
#define DOCKING_GUIDANCE_GLOBLEDATA_H

#define LOGID_I "info"
#define LOGID_C "custom"
#define LOGID_B "bin"



void  init_log();


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

} ;

//消息队列的声明
struct my_msg_st {
    long int my_msg_type;
    char some_text[1024];
};







#endif //DOCKING_GUIDANCE_GLOBLEDATA_H
