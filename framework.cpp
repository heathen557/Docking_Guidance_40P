//
// Created by LKK on 18-6-23.
//
#include "framework.h"
#include <math.h>

#include<sys/socket.h>
#include<sys/types.h>
#include<sys/un.h>
#include <unistd.h>
#include <linux/tcp.h>
#include <netinet/in.h>

#include "GlobleData.h"

#define UNIX_DOMAIN_SEND "/tmp/LidarToManage.domain"
#define UNIX_DOMAIN_RECV "/tmp/ManageToLidar.domain"



using namespace rapidjson;

#define DISPLAYPORT 9999
//#define DISPLAYIP "192.168.1.118"
#define DISPLAYIP "127.0.0.1"
#define BUFSIZE 512
#define FALSE 0

std::mutex _mtx;
int _READYTOSENDFLAG = 0;
DISPLAYINFO _displayinfo;
int _workstatus = 0 ;
extern struct control_msg con_msg;

//UDP套接字发送数据
void socket_UDP() {

    int client_sockfd;
    char buf[BUFSIZE];
    int len;
    struct sockaddr_in serveraddr;

    std::memset(buf, 0, BUFSIZE);
    std::memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(DISPLAYPORT);
    serveraddr.sin_addr.s_addr = inet_addr(DISPLAYIP);

    std::cout << "here" << std::endl;
    if ((client_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cout << "socket error" << std::endl;
        exit(FALSE);
    }

    Document doc;
    doc.SetObject();
    Document::AllocatorType &allocator = doc.GetAllocator(); //获取分配器

    /*initialzation*/
    _displayinfo.craft = "A380";
    _displayinfo.distance = 50.0;
    _displayinfo.speed = 5.31;
    _displayinfo.position = "left";
    _displayinfo.offset = 3.0;

    Value flight, position;
    char buf_tmp[20];
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
    flight.SetString(buf_tmp, len_craft, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
    position.SetString(buf_tmp, len_position, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));
    doc.AddMember("craft", flight, allocator);
    doc.AddMember("distance", _displayinfo.distance, allocator);
    doc.AddMember("speed", _displayinfo.speed, allocator);
    doc.AddMember("position", position, allocator);
    doc.AddMember("offset", _displayinfo.offset, allocator);
    doc.AddMember("detectflag", _displayinfo.detectflag, allocator);
    int i = 0;
    while (1) {
//        std::cout<<"the flag : " <<_READYTOSENDFLAG<<std::endl;
        if (_READYTOSENDFLAG) {
//            std::cout<<"ready"<<std::endl;
            _mtx.lock();
            _READYTOSENDFLAG = 0;
            _mtx.unlock();
            //displayinfo.craft = "B777";
            //displayinfo.position = "haha";
            //_displayinfo.distance = i%40;
            //i++;
            len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
            flight.SetString(buf_tmp, len_craft, allocator);
            doc["craft"] = flight;
            int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
            position.SetString(buf_tmp, len_position, allocator);
            memset(buf_tmp, 0, sizeof(buf_tmp));
            doc["position"] = position;
            _displayinfo.distance = round(_displayinfo.distance * 100) / 100;
            doc["distance"] = _displayinfo.distance;
            doc["speed"] = _displayinfo.speed;
            doc["offset"] = _displayinfo.offset;
            doc["detectflag"] = _displayinfo.detectflag;
            StringBuffer buffer;
            Writer<StringBuffer> writer(buffer);
            doc.Accept(writer);

            std::cout << buffer.GetString() << std::endl;

            len = sizeof(struct sockaddr_in);

            if ((len = sendto(client_sockfd, buffer.GetString(), strlen(buffer.GetString()), 0,
                              (struct sockaddr *) &serveraddr, sizeof(struct sockaddr))) < 0) {
                perror("sendto");
                exit(FALSE);
            }
        } else {
            usleep(100 * 1000);
        }
    }
    close(client_sockfd);
}


//本地socket发送数据
void Send_localsocket(){


    std::cout << "本地发送socket已经开启了！" << std::endl;

    int connect_fd;
    int ret;
    char send_buff[1024];
    static struct sockaddr_un srv_addr;

    //--------------
//    int len_craft;
//    char buf_tmp[20];
//    Value flight, position;

    Document doc;
    doc.SetObject();
    Document::AllocatorType &allocator = doc.GetAllocator(); //获取分配器
    /*initialzation*/
    _displayinfo.craft = "A380";
    _displayinfo.distance = 50.0;
    _displayinfo.speed = 5.31;
    _displayinfo.position = "left";
    _displayinfo.offset = 3.0;

    Value flight, position;
    char buf_tmp[20];
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
    flight.SetString(buf_tmp, len_craft, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));
    int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
    position.SetString(buf_tmp, len_position, allocator);
    memset(buf_tmp, 0, sizeof(buf_tmp));


    doc.AddMember("@table",1,allocator);
    doc.AddMember("@src","lidar",allocator);
    doc.AddMember("workstatus",_workstatus,allocator);
    doc.AddMember("detectflag", _displayinfo.detectflag, allocator);
    doc.AddMember("craft", flight, allocator);
    doc.AddMember("position", position, allocator);
    doc.AddMember("distance", _displayinfo.distance, allocator);
    doc.AddMember("speed", _displayinfo.speed, allocator);
    doc.AddMember("offset", _displayinfo.offset, allocator);


    //定义发送消息队列变量
    int msgid_2;
    struct my_msg_st some_data_2;
    long int msg_to_receive_2 = 0;
    //创建消息队列
    msgid_2 = msgget((key_t) 1235, 0666 | IPC_CREAT);
    if (msgid_2 == -1) {
        fprintf(stderr, "发送线程 接收消息队列msgget failed with error: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    while (1) {


        //接收到主线程的返回信息，并将相关信息发送给前端；
        if (msgrcv(msgid_2, (void *) &some_data_2, BUFSIZ, msg_to_receive_2, IPC_NOWAIT) == -1)
        {
//            fprintf(stderr, "msgrcv failed with error: %d\n", errno);
//            exit(EXIT_FAILURE);
        } else
        {
//            printf("发送线程接收到的消息队列的数据为： %s\n", some_data_2.some_text);
            std::cout << "发送线程接收到的消息队列的数据为： %s\n" << some_data_2.some_text << std::endl;

            connect_fd=socket(PF_UNIX,SOCK_STREAM,0);
            if(connect_fd<0){
                perror("cannot creat socket");
                zlog_error(c, "发送时初始化本地socket失败！！");
                break;
            }
            srv_addr.sun_family=AF_UNIX;
            strcpy(srv_addr.sun_path,UNIX_DOMAIN_SEND);


            ret=connect(connect_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
            if (ret<0){
//          perror("cannot connect server");
                printf("cannot connect to 控制软件\n");
                zlog_error(c, "cannot connect to 控制软件\n");
                close(connect_fd);
//            break;
                continue;
            }

            write(connect_fd,some_data_2.some_text, sizeof(some_data_2.some_text));
            close(connect_fd);
            continue;


        }

        //发送算法的结果数据



        if (_READYTOSENDFLAG)
        {
            std::cout << "发送本地socket时,标识位~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

            _READYTOSENDFLAG = 0;


            connect_fd=socket(PF_UNIX,SOCK_STREAM,0);
            if(connect_fd<0){
                perror("cannot creat socket");
                break;
            }
            srv_addr.sun_family=AF_UNIX;
            strcpy(srv_addr.sun_path,UNIX_DOMAIN_SEND);


            ret=connect(connect_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
            if (ret<0){
                zlog_warn(c, "未能链接到服务器 ，退出发送");
                std::cout << "未能链接到服务器 ，退出发送" << std::endl;
                close(connect_fd);
//                continue;
            }

            //---------------------------------------------
            len_craft = sprintf(buf_tmp, "%s", _displayinfo.craft.c_str());
            flight.SetString(buf_tmp, len_craft, allocator);
            doc["craft"] = flight;
            int len_position = sprintf(buf_tmp, "%s", _displayinfo.position.c_str());
            position.SetString(buf_tmp, len_position, allocator);
            memset(buf_tmp, 0, sizeof(buf_tmp));
            doc["position"] = position;
            _displayinfo.distance = round(_displayinfo.distance * 100) / 100;
            doc["distance"] = _displayinfo.distance;
            doc["speed"] = _displayinfo.speed;
            doc["offset"] = _displayinfo.offset;
            doc["detectflag"] = _displayinfo.detectflag;
            doc["workstatus"] = _workstatus;
            StringBuffer buffer;
            Writer<StringBuffer> writer(buffer);
            doc.Accept(writer);

            std::cout << "上传的数据为：" << buffer.GetString() << std::endl;
//            LOG__(LOGID_I, "要发送的数据为： %s\n", buffer.GetString());
            zlog_debug(c, "上传的数据为：%s", buffer.GetString());



            //---------------------------------------------
//            char *ch = "{\"craft\":\"WalkTest\",\"distance\":27.0,\"speed\":0.0,\"position\":\"LEFT\",\"offset\":1.3835633748444607,\"detectflag\":1}";
//            memset(send_buff,0,1024);
//            strcpy(send_buff,ch);
            //send info server
            write(connect_fd,buffer.GetString(),strlen(buffer.GetString()));
            close(connect_fd);
        }

        usleep(1000*100);
    }


}




//struct my_msg_st {
//    long int my_msg_type;
//    char some_text[1024];
//};

//udp套接字接收数据
void Recvsocket_UDP() {
    std::cout << "接收数据的函数已经进来了";


    int server_sockfd;
    int len;
    struct sockaddr_in my_addr;             //服务器网络地址结构体
    struct sockaddr_in remote_addr;         //客户端网络地址结构体
    int sin_size;
    char buf[1024];                         //数据传送的缓冲区
    memset(&my_addr, 0, sizeof(my_addr));   //数据初始化--清零
    my_addr.sin_family = AF_INET;           //设置为IP通信
    my_addr.sin_addr.s_addr = INADDR_ANY;   //服务器IP地址--允许连接到所有本地地址上
    my_addr.sin_port = htons(8000);         //服务器端口号

    //创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议
    if ((server_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        return;
    }

    //将套接字绑定到服务器的网络地址上
    if (bind(server_sockfd, (struct sockaddr *) &my_addr, sizeof(struct sockaddr)) < 0) {
        perror("bind");
        return;
    }
    sin_size = sizeof(struct sockaddr_in);
    printf("waiting for a packet...\n");

    /*接收客户端的数据并将其发送给客户端--recvfrom是无连接的*/

    /////////////////////消息队列的初始化///////////
    struct my_msg_st some_data;
    int msgid;
    char buffer[1024];

    //创建消息队列
    msgid = msgget((key_t) 1234, 0666 | IPC_CREAT);
    if (msgid == -1) {
        fprintf(stderr, "msgget failed with error:%d\n", errno);
        exit(EXIT_FAILURE);
    }


    while (1) {
        std::memset(buf, 0, sizeof(buf) - 1);
        len = recvfrom(server_sockfd, buf, sizeof(buf) - 1, 0, (struct sockaddr *) &remote_addr,
                       (socklen_t *) &sin_size);
        if (len < 0) {
            perror("recvfrom");
            return;
        }
        buf[len] = '\0';
        printf("received packet from %s:\n", inet_ntoa(remote_addr.sin_addr));
        printf("contents: %s \n", buf);

        //写入到消息队列
        some_data.my_msg_type = 1;
        strcpy(some_data.some_text, buf);
        if (msgsnd(msgid, (void *) &some_data, 1024, 0) == -1) {
            fprintf(stderr, "msgsed failed\n");
            exit(EXIT_FAILURE);
        }
        usleep(100 * 1000);
    }
    close(server_sockfd);
}



//本地socket接收数据
void Recv_localSocket()
{
    socklen_t clt_addr_len;
    int listen_fd;
    int com_fd;
    int ret;
    int i;
    static char rcv_buff[1024];
    int len;
    struct sockaddr_un clt_addr;
    struct sockaddr_un srv_addr;
    srv_addr.sun_family=AF_UNIX;
    strncpy(srv_addr.sun_path,UNIX_DOMAIN_RECV,sizeof(srv_addr.sun_path)-1);
    unlink(UNIX_DOMAIN_RECV);

    /////////////////////消息队列的初始化///////////
    struct my_msg_st some_data;
    int msgid;
    char buffer[1024];
    //创建消息队列
    msgid = msgget((key_t) 1234, 0666 | IPC_CREAT);
    if (msgid == -1) {
        fprintf(stderr, "msgget failed with error:%d\n", errno);
        exit(EXIT_FAILURE);
    }



    while (1) {

        listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
        if(listen_fd<0){
            perror("connect creat communication socket");
        }

        //bind sockfd&addr
        ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
        if(ret<0){
            perror("cannot bind server socket");
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //listen sockfd
        ret=listen(listen_fd,1);
        if(ret<0){
            perror("cannot listen sockfd");
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //have connect requst use accept
        len=sizeof(clt_addr);
        com_fd=accept(listen_fd,(struct sockaddr*)&clt_addr,(socklen_t *)&len);
        if(com_fd<0){
            perror("cannot accept requst " ) ;
            close(listen_fd);
            unlink(UNIX_DOMAIN_RECV);
            break;
        }

        //read and printf client send info
        for(i=0;i<4;i++){
            memset(rcv_buff,0,1024);
            int num = read(com_fd,rcv_buff,sizeof(rcv_buff));
            if(0 < num)
            {
                printf("message from client %d : %s\n",num,rcv_buff);

                //写入到消息队列
                some_data.my_msg_type = 1;
                strcpy(some_data.some_text, rcv_buff);
                if (msgsnd(msgid, (void *) &some_data, 1024, 0) == -1) {
                    fprintf(stderr, "msgsed failed\n");
                    exit(EXIT_FAILURE);
                }
            }

        }
        close(com_fd);
        close(listen_fd);
        unlink(UNIX_DOMAIN_RECV);
        usleep(1000*1000);

    }

    return ;
}
