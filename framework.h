#ifndef FRAMEWORK_H_
#define FRAMEWORK_H_

#include <iostream>
#include <cstring>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <unistd.h>
#include <zlog.h>


#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

typedef struct DISPLAYINFO{
    std::string craft;
    double distance;
    double speed;
    std::string position;
    double offset;
    int detectflag;
    int obstacledetection;   //  0 :无障碍 1:有障碍
    int occlusiondetect;        //  0 :无遮挡 1:有遮挡
}DISPLAYINFO;

typedef struct SELFCHECK {
    int obstacle_flag;       //  0 :无障碍 1:有障碍
    int occlusion_flag;      //  0 :无遮挡 1:有遮挡
} SELFCHECKINFO;

extern zlog_category_t *c;
extern std::mutex _mtx;
extern int _READYTOSENDFLAG;
extern DISPLAYINFO _displayinfo;
extern SELFCHECKINFO _selfCheckinfo;



void socket_UDP();

void Recvsocket_UDP();    //接受数据模块

void Recv_localSocket();

void Send_localsocket();

#endif /*FRAMEWORK_H_*/