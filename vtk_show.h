//
// Created by he on 19-3-4.
//



#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include "zlog.h"

#include "Cluster.h"
#include "tools_function.h"

#ifndef DOCKING_GUIDANCE2_VTK_SHOW_H
#define DOCKING_GUIDANCE2_VTK_SHOW_H

zlog_category_t *c;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
bool succed_target_;
CloudPtr src_cloud_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud_;
boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler_("intensity");

int show_personOrAircarft = 1;    //  1:person(default)    2:aircaft



CloudPtr front_aircraft_;
CloudPtr side_aircraft_;


void show_pcl()
{
    cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    std::cout<<"显示点云数据的线程已经开启了,here～～～～～～～～～～～"<<std::endl;

//    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
    while (!cloud_viewer_->wasStopped())
    {
        if (1 == show_personOrAircarft)     //显示行人检测数据
        {
            if (succed_target_) {
                if (!cloud_viewer_->updatePointCloud(drawed_cloud_)) {
                    cloud_viewer_->addPointCloud(drawed_cloud_);
                }
            } else {
                if (src_cloud_) {
                    handler_.setInputCloud(src_cloud_);
                    if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI>(src_cloud_, handler_)) {
                        cloud_viewer_->addPointCloud<pcl::PointXYZI>(src_cloud_, handler_);
                    }
                }
            }
        } else if (2 == show_personOrAircarft)   //显示飞机检测数据
        {
            if (front_aircraft_)//succed_detect_head_
            {
                handler_.setInputCloud(front_aircraft_);
                if (!cloud_viewer_->updatePointCloud(front_aircraft_, handler_)) {
                    cloud_viewer_->addPointCloud(front_aircraft_, handler_);
                }
            } else {
                if (side_aircraft_)//succed_detect_side_
                {
                    handler_.setInputCloud(side_aircraft_);
                    if (!cloud_viewer_->updatePointCloud(side_aircraft_, handler_)) {
                        cloud_viewer_->addPointCloud(side_aircraft_, handler_);
                    }
                } else {
                    if (src_cloud_) {
                        handler_.setInputCloud(src_cloud_);
                        if (!cloud_viewer_->updatePointCloud(src_cloud_, handler_)) {
                            cloud_viewer_->addPointCloud(src_cloud_, handler_);
                        }
                    }
                }
            }
        }

        cloud_viewer_->spinOnce(1, true);


    }
}



class vtk_show {

};


#endif //DOCKING_GUIDANCE2_VTK_SHOW_H
