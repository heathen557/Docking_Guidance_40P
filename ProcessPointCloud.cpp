//
// Created by luffy7n on 18-10-30.
//
#include "ProcessPointcloud.h"
float calculatePointDistance(PointType p1, pcl::PointXYZ p2)
{
    float x_d = p1.x - p2.x;
    float y_d = p1.y - p2.y;
    float z_d = p1.z - p2.z;
    float distance = sqrt(x_d*x_d + y_d*y_d +z_d*z_d);
    return distance;
}

float calculateSideLength(const CloudPtr &in_cloud_ptr)
{
    float min_x = in_cloud_ptr->points[0].x;
    float max_x = in_cloud_ptr->points[0].x;
    int min_id = 0;
    int max_id = 0;
    for (int i = 1; i < in_cloud_ptr->size(); ++i)
    {
        if (min_x > in_cloud_ptr->points[i].x)
        {
            min_x = in_cloud_ptr->points[i].x;
            min_id = i;
        }
        if (max_x < in_cloud_ptr->points[i].x)
        {
            max_x = in_cloud_ptr->points[i].x;
            max_id = i;
        }
    }
    float right_y = in_cloud_ptr->points[min_id].y;
    float left_y = in_cloud_ptr->points[max_id].y;
    float side_length = sqrtf((max_x-min_x)*(max_x-min_x) + (left_y-right_y)*(left_y-right_y));
    return side_length;
}

void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr)
{
    out_cloud_ptr->width = in_cloud_ptr->size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->resize(out_cloud_ptr->width * out_cloud_ptr->height);
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        out_cloud_ptr->points[i].x = in_cloud_ptr->points[i].x;
        out_cloud_ptr->points[i].y = in_cloud_ptr->points[i].y;
        out_cloud_ptr->points[i].z = in_cloud_ptr->points[i].z;
    }
}

PointType findMinValues(const CloudPtr &in_cloud_ptr)
{
    PointType point;
    point.x = in_cloud_ptr->points[0].x;
    point.y = in_cloud_ptr->points[0].y;
    point.z = in_cloud_ptr->points[0].z;

    for (size_t i = 1; i < in_cloud_ptr->points.size(); ++i)
    {
        if (in_cloud_ptr->points[i].z < point.z)
            point.z = in_cloud_ptr->points[i].z;
        if (in_cloud_ptr->points[i].y < point.y)
            point.y = in_cloud_ptr->points[i].y;
        if (in_cloud_ptr->points[i].x < point.x)
            point.x = in_cloud_ptr->points[i].x;
    }
    return point;
}

PointType findMaxValues(const CloudPtr &in_cloud_ptr)
{
    PointType point;
    point.x = in_cloud_ptr->points[0].x;
    point.y = in_cloud_ptr->points[0].y;
    point.z = in_cloud_ptr->points[0].z;

    for (size_t i = 1; i < in_cloud_ptr->points.size(); ++i)
    {
        if (in_cloud_ptr->points[i].z > point.z)
            point.z = in_cloud_ptr->points[i].z;
        if (in_cloud_ptr->points[i].y > point.y)
            point.y = in_cloud_ptr->points[i].y;
        if (in_cloud_ptr->points[i].x > point.x)
            point.x = in_cloud_ptr->points[i].x;
    }
    return point;
}

CloudPtr getCloudByIndices(const CloudPtr &in_cloud_ptr, vector<int> cloud_indices)
{
    CloudPtr out_cloud_ptr;
    out_cloud_ptr.reset(new Cloud);
    out_cloud_ptr->width = cloud_indices.size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->points.resize(out_cloud_ptr->width * out_cloud_ptr->height);
    for (int i = 0; i < cloud_indices.size(); ++i)
    {
        out_cloud_ptr->points[i].x = in_cloud_ptr->points[cloud_indices[i]].x;
        out_cloud_ptr->points[i].y = in_cloud_ptr->points[cloud_indices[i]].y;
        out_cloud_ptr->points[i].z = in_cloud_ptr->points[cloud_indices[i]].z;
    }
    return out_cloud_ptr;
}

void passthroughCloud(const CloudPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_limit,
                      float max_limit, std::string &field_name, bool select)
{
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(min_limit, max_limit);
    pass.setFilterLimitsNegative(select);
    pass.setInputCloud(in_cloud_ptr);
    pass.filter(*out_cloud_ptr);
}

void setTargetColor(const CloudConstPtr &in_cloud_ptr, std::vector<int> &in_cluster_indices,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr, int r, int g, int b)
{
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = in_cloud_ptr->points[i].x;
        p.y = in_cloud_ptr->points[i].y;
        p.z = in_cloud_ptr->points[i].z;
        std::vector<int>::iterator iter = std::find(in_cluster_indices.begin(), in_cluster_indices.end(), i);
        if (iter == in_cluster_indices.end())
        {
            p.r = 250;
            p.g = 250;
            p.b = 250;
        }
        else
        {
            p.r = r;
            p.g = g;
            p.b = b;
        }
        out_cloud_ptr->push_back(p);
    }
}

void removePointsUpto(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr,
                      const float remove_points_upto)
{
    out_cloud_ptr->points.clear();
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        float orign_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
        if (orign_distance > remove_points_upto)
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}

void clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_x, float max_x, float min_y,
               float max_y) // 后考虑与下面的clipCloud合并
{
    out_cloud_ptr->points.clear();
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++) {
        if (in_cloud_ptr->points[i].x > min_x && in_cloud_ptr->points[i].x < max_x && in_cloud_ptr->points[i].y > min_y
            && in_cloud_ptr->points[i].y < max_y) {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}

void clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, bool direction, float min_height,
               float max_height, float right_pos, float left_pos, float bottom_pos)
{
    out_cloud_ptr->points.clear();
    if (direction)
    {
        for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
        {
            if (in_cloud_ptr->points[i].z > min_height && in_cloud_ptr->points[i].z < max_height && in_cloud_ptr->points[i].x > right_pos &&
                in_cloud_ptr->points[i].x < left_pos && in_cloud_ptr->points[i].y > bottom_pos)//
            {
                out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
        {
            if (in_cloud_ptr->points[i].z > min_height && in_cloud_ptr->points[i].z < max_height && in_cloud_ptr->points[i].x > right_pos &&
                in_cloud_ptr->points[i].x < left_pos && in_cloud_ptr->points[i].y < bottom_pos)//
            {
                out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
            }
        }
    }
}

void removeFloor(const CloudPtr &in_cloud_ptr, CloudPtr out_nofloor_cloud_ptr,
                 CloudPtr out_onlyfloor_cloud_ptr, float in_max_height, float in_floor_max_angle)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);//SACMODEL_PERPENDICULAR_PLANE
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(in_floor_max_angle);

    seg.setDistanceThreshold(in_max_height);
    //seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    //REMOVE THE FLOOR FROM THE CLOUD
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);//true removes the indices, false leaves only the indices
    extract.filter(*out_nofloor_cloud_ptr);

    //EXTRACT THE FLOOR FROM THE CLOUD
    extract.setNegative(false);//true removes the indices, false leaves only the indices
    extract.filter(*out_onlyfloor_cloud_ptr);
}

vector<pcl::PointIndices> regiongrowingSegmentation(const CloudPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in_cloud_ptr);
    normal_estimator.setKSearch(10); //50
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(30); //100
    reg.setMaxClusterSize(60); //1000
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(in_cloud_ptr);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1.5 * M_PI);
    reg.setCurvatureThreshold(0.5);
    reg.extract(cluster_indices);

    //std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;

    return cluster_indices;
}

vector<pcl::PointIndices> euclideanCluster(const CloudPtr &in_cloud_ptr, int cluster_size_min,
                                           int cluster_size_max, float cluster_tolerance)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud(in_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance); //2*voxel_size 0.2
    ec.setMinClusterSize(cluster_size_min);
    ec.setMaxClusterSize(cluster_size_max);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(cluster_indices);
    //int size = cluster_indices.size();

    //std::cout << "The " << cloud_id_ << " frame's number of clusters is equal to " << cluster_indices.size() << std::endl;

    //For every cluster...

    return cluster_indices;
}

bool detectCircle(const CloudPtr &in_cloud_ptr, float min_radius, float max_radius,
                  Eigen::VectorXf *coefficients, CloudPtr out_cloud_ptr)
{
    std::vector<int> inliers_indicies;
    pcl::SampleConsensusModelCircle3D<PointType>::Ptr model_circle(
            new pcl::SampleConsensusModelCircle3D<PointType>(in_cloud_ptr));
    pcl::RandomSampleConsensus<PointType>ransac_circle(model_circle);
    ransac_circle.setDistanceThreshold(0.1);
    model_circle->setRadiusLimits(min_radius, max_radius);
    ransac_circle.computeModel();
    inliers_indicies.clear();
    ransac_circle.getInliers(inliers_indicies);
    if (inliers_indicies.size() == 0)
    {
        std::cout << "Can not detect circle" << std::endl;
        return false;
    }
    else
    {
        ransac_circle.getModelCoefficients(*coefficients);
        pcl::copyPointCloud<PointType>(*in_cloud_ptr, inliers_indicies, *out_cloud_ptr);
        return true;
    }
}

bool findTarget(const CloudPtr &in_cloud_ptr, PointType centre, float radius, vector<int> *cloud_indices)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    vector<float> point_radius_square_distance;
    if (in_cloud_ptr->empty())
    {
        return false;
    }
    //float radius = 1.2;// 可根据速度设定
    kdtree.setInputCloud(in_cloud_ptr); //确定？
    kdtree.radiusSearch(centre, radius, *cloud_indices, point_radius_square_distance);
    if (cloud_indices->size() > 0)
    {
        std::cout << "search: " << cloud_indices->size() << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

bool getCloudFromCluster(const CloudPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices, vector<ClusterPtr> *target)
{
    int cluster_id = 0;
    //vector<ClusterPtr> clusters;
    for (vector<pcl::PointIndices>::iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i)
    {
        ClusterPtr cluster(new Cluster());
        cluster->setCloud(in_cloud_ptr, i->indices, cluster_id, 255, 0, 0);
        if (fabs(cluster->getCentroid().y) > 50) //后加利用强度信息
        {
            target->push_back(cluster);
        }
        cluster_id++;
    }
    if (target->size() != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool getCloudFromCluster(const CloudPtr &in_cloud_ptr, vector<pcl::PointIndices> cluster_indices,
                         float target_min_height, float target_max_height,
                         float target_min_width, float target_max_width,
                         vector<ClusterPtr> *target)//std::vector<int> *target_indices)  //默认参数变量如何写
{
    int cluster_id = 0;
    //std::vector<Cloud> clustered_clouds;
    //std::vector<ClusterPtr> clusters;
    //std::vector<ClusterPtr> target;
    CloudPtr cluster_cloud(new Cloud);
    CloudPtr all_cluster_cloud(new  Cloud);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud;
    for (std::vector<pcl::PointIndices>::iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i)
    {
        ClusterPtr cluster(new Cluster());
        cluster->setCloud(in_cloud_ptr, i->indices, cluster_id, 255, 0, 0); //, (int)colors_[cluster_id].val[0], (int)colors_[cluster_id].val[1], (int)colors_[cluster_id].val[2]);
        transformPointCloud(cluster->getCloud(), cluster_cloud);
        *all_cluster_cloud += *cluster_cloud;
        cluster_cloud->clear();
        //CreateInputFile(cluster);   //for train
        if (cluster->getHeight() > target_min_height && cluster->getHeight() < target_max_height &&cluster->getWidth() > target_min_width && cluster->getWidth() < target_max_width)// && (0.7,0.3)　//
        {
            //*target_indices = i->indices;
            target->push_back(cluster);
            float x = cluster->getCentroid().x;
            float y = cluster->getCentroid().y;
            float z = cluster->getCentroid().z;
            //std::cout << "frame " << cloud_id_ << " x coordinate is: " << x << std::endl;
            std::cout << "The coordinate of target is (" << x << " " << y << " " << z << ")" << std::endl;
        }
        //clusters.push_back(cluster);
        cluster_id++;
    }
    if (target->size() != 0)
    {
        cout << "检测到符合几何参数的目标的个数为 " << target->size() << endl;
        zlog_info(c, "检测到符合几何参数的目标的个数为:%d \n ", target->size());
        //all_cluster_cloud_ = all_cluster_cloud;
        return true;
    }
    else
    {
        //cout << cloud_id_ <<"th Frame has None Target :" << endl;
        return false;
    }
}

