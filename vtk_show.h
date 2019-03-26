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

//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/segmentation/region_growing.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/sac_model_circle3d.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/ia_ransac.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_rejection_features.h>
//#include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/approximate_voxel_grid.h>


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

void show_pcl()
{
    cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    std::cout<<"显示点云数据的线程已经开启了,here～～～～～～～～～～～"<<std::endl;


//    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
    while (!cloud_viewer_->wasStopped())
    {
        if (succed_target_)
        {
            if (!cloud_viewer_->updatePointCloud(drawed_cloud_))
            {
                cloud_viewer_->addPointCloud(drawed_cloud_);
            }
        }
        else
        {
            if (src_cloud_)
            {
                handler_.setInputCloud(src_cloud_);
                if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI>(src_cloud_, handler_))
                {
                    cloud_viewer_->addPointCloud<pcl::PointXYZI>(src_cloud_, handler_);
                }
            }
        }
        cloud_viewer_->spinOnce(1, true);


    }
}



class vtk_show {

};


#endif //DOCKING_GUIDANCE2_VTK_SHOW_H
