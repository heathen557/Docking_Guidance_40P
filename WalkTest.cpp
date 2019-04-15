//
// Created by luffy7n on 18-11-5.
//

#include "zlog.h"
#include "WalkTest.h"
#include "GlobleData.h"
extern struct control_msg con_msg;
extern bool _run_flag;
extern int _workstatus;

extern zlog_category_t *c;
WalkTest *WalkTest::pThis_ = NULL;

void WalkTest::setParameter()
{
    check_mode_ = con_msg.detectionModel;   //0:自检模式   1：目标检测过程中的自检
    delay_count = 0;
    origin_distance_ = 150;
    end_distance_= 120;
    offset_ = 0;
    x_precision_ = 0.0; //偏离中线的容忍度
    y_precision_ = 0.0; //距离终止线的容忍度
    remove_points_upto_ = 0;
    clip_min_height_ = -20;
    clip_max_height_ = 2;
    //clip_right_pos_ = con_msg.end_point1_x;//-0.7;//-0.7;
    //clip_left_pos_ = con_msg.end_point0_x;//0.9;//0.9;
    if (con_msg.end_point1_x < con_msg.end_point0_x)
    {
        clip_right_pos_ = con_msg.end_point1_x;//-0.7;//-0.7;
        clip_left_pos_ = con_msg.end_point0_x;//0.9;//0.9;
    }
    else
    {
        clip_right_pos_ = con_msg.end_point0_x;
        clip_left_pos_ = con_msg.end_point1_x;
    }
    //clip_bottom_pos_ = end_point0_.y + 1.5;
    cluster_size_min_ = 30;//70;//_minPoints;
    cluster_size_max_ = 500;//2000;//_maxPoints;
    search_radius_ = 1.2; // 再优化
    cluster_tolerance_ = 0.4;
    target_min_height_ = 0.5;
    target_max_height_ = 1.9;
    target_min_width_ = 0.3;
    target_max_width_ = 0.9;
    region_points_size_ = con_msg.detecPointsSize;            //检测点个数
    for (int i = 0; i < region_points_size_; ++i) {
        region_x_[i] = con_msg.detecPointX[i];
        region_y_[i] = con_msg.detecPointY[i];
    }
    mild_point0_ = {con_msg.mild_point0_x, con_msg.mild_point0_y};//0.01, -4.83;
    mild_point1_ = {con_msg.mild_point1_x, con_msg.mild_point1_y};//0.28, -34.6
    end_point0_ = {con_msg.end_point0_x, con_msg.end_point0_y};//1.03, -4.88
    end_point1_ = {con_msg.end_point1_x, con_msg.end_point1_y};//-0.77, -4.9
    right_point0_ = {mild_point0_.x-3.5, mild_point0_.y};
    right_point1_ = {mild_point1_.x-3.5, mild_point1_.y};
    left_point0_ = {mild_point0_.x+3.5, mild_point0_.y};
    left_point1_ = {mild_point1_.x+3.5, mild_point1_.y};
    if (end_point0_.y > mild_point0_.y || end_point0_.y > mild_point1_.y)
    {
        direction_ = 0;
        clip_bottom_pos_ = end_point0_.y + 2;
    }
    else
    {
        direction_ = 1;
        clip_bottom_pos_ = end_point0_.y - 2;
    }
    mild_p0l_ = {con_msg.mild_point0_x+0.03f, con_msg.mild_point0_y, -6};
    mild_p1l_ = {con_msg.mild_point1_x+0.03f, con_msg.mild_point1_y, -6};
    end_p0u_ = {con_msg.end_point0_x, con_msg.end_point0_y-0.03f, -6};
    end_p1u_ = {con_msg.end_point1_x, con_msg.end_point1_y-0.03f, -6};
    mild_p0r_ = {con_msg.mild_point0_x-0.03f, con_msg.mild_point0_y, -6};
    mild_p1r_ = {con_msg.mild_point1_x-0.03f, con_msg.mild_point1_y, -6};
    end_p0d_ = {con_msg.end_point0_x, con_msg.end_point0_y+0.03f, -6};
    end_p1d_ = {con_msg.end_point1_x, con_msg.end_point1_y+0.03f, -6};
    mild_p0_ = {con_msg.mild_point0_x, con_msg.mild_point0_y, -6};
    mild_p1_ = {con_msg.mild_point1_x, con_msg.mild_point1_y, -6};
    end_p0_ = {con_msg.end_point0_x, con_msg.end_point0_y, -6};
    end_p1_ = {con_msg.end_point1_x, con_msg.end_point1_y, -6};
    model_feature_.push_back(1.7);
    model_feature_.push_back(0.5);
}

void WalkTest::initializeParameter()
{
    cloud_id_ = 0;
    lost_counter_ = 21;
    detect_flag_ = false;
    src_cloud_.reset(new Cloud);
    prev_cloud_.reset(new Cloud);
    nofloor_cloud_.reset(new Cloud);
    pre_target_indices_.clear();
    pre_target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    model_cloud_.reset(new Cloud);
    //remove_points_upto_ = 0; // 設定2？
    remove_ground_ = true;
    use_region_growing_ = false;
    succed_detect_ = false;
    //priority_ = 0;
    succed_target_ = false;
    succed_detect_counter_ = 0;
    addline_ = true;
    track_target_cluster_.reset(new Cluster());
    pcl::io::loadPCDFile("model.pcd", *model_cloud_);
    pre_target_centroid_.x = 0;
    pre_target_centroid_.y = 0;
    pre_target_centroid_.z = 0;
    pre_end_distance_ = 0;
    pre_offset_ = 0;
    pre_position_ = "MILD";

    //cloud_viewer_->reset(new pcl::visualization::CloudViewer("3D Viewer"));
    //cloud_viewer_("3D Viewer");
    //cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    for (int i = 0; i < 15; ++i)
    {
        string rightline_id = "rightline" + to_string(i);
        string leftline_id = "leftline" + to_string(i);
        pcl::PointXYZ right_p0(right_point0_.x, right_point0_.y, -6+i*0.5);
        pcl::PointXYZ right_p1(right_point1_.x, right_point1_.y, -6+i*0.5);
        pcl::PointXYZ left_p0(left_point0_.x, left_point0_.y, -6+i*0.5);
        pcl::PointXYZ left_p1(left_point1_.x, left_point1_.y, -6+i*0.5);
        cloud_viewer_->addLine(right_p0, right_p1, 0, 0, 255, rightline_id); //
        cloud_viewer_->addLine(left_p0, left_p1, 0, 0, 255, leftline_id);//
    }

    cloud_viewer_->addLine(mild_p0_, mild_p1_, "mildline");
    cloud_viewer_->addLine(mild_p0l_, mild_p1l_, "mildline1");
    cloud_viewer_->addLine(mild_p0r_, mild_p1r_, "mildline2");
    cloud_viewer_->addLine(end_p0_, end_p1_, "endline");
    cloud_viewer_->addLine(end_p0u_, end_p1u_, "endline1");
    cloud_viewer_->addLine(end_p0d_, end_p1d_, "endline2");
}

WalkTest::WalkTest(std::string &calib_file, std::string &pcap_file,
                   pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
        : calibration_file_(calib_file)
        , pcap_file_(pcap_file)
        //, handler_(handler)
        //, cloud_viewer_("3D viewer")
        //, interface_(calibration_file_, pcap_file_)
        , pandar40p_(ip_, 2368, 10110, lidarCallback, gpsCallback, 15000, 0,  string("hesai40"))
{
    pThis_ = this;
    //cv::generateColors(colors_, 100);
    setParameter();
    initializeParameter();

}

WalkTest::WalkTest(std::string &calib_file, std::string &ip, int port,
                   pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
        : calibration_file_(calib_file), ip_(ip), port_(port)
//, handler_(handler)
//, cloud_viewer_("3D viewer")
//, interface_(boost::asio::ip::address::from_string(ip_), port_, calibration_file_)
        , pandar40p_(ip_, 2368, 10110, lidarCallback, gpsCallback, 15000, 0, string("hesai40"))
{
    std::cout<<"行人检测的构造函数已经进来了"<<std::endl;

    pThis_ = this;
    //cv::generateColors(colors_, 100);
    setParameter();
    //cout << "ok.............." << endl;
    initializeParameter();

    localfileTest();

}

void WalkTest::localfileTest() {
    int i = 0;

    while (1) {
        drawed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        i++;
        string fileName = "12-25/" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_per(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile(fileName, *cloud_per) == -1) {
            zlog_error(c, "pcd路径出错，或者文件读取完毕！！");
            std::cout << "pcd路径出错，或者文件读取完毕！！" << std::endl;
            break;
        }
//        zlog_debug(c,"第 %d   个pcd文件的大小为：%d,",i,cloud_per->size());
        pThis_->flowCloud(cloud_per);

        usleep(10000);
    }
}



vector<float> WalkTest::extractClusterFeature(ClusterPtr cluster)
{
    vector<float> feature;
    feature.clear();
    float height = cluster->getHeight();
    float width = cluster->getWidth();
    feature.push_back(height);
    feature.push_back(width);
    return feature;
}

FpfhFeaturePtr WalkTest::computeFpfhFeature(const CloudPtr &in_cloud_ptr, KdtreePtr tree)
{
    //法向量
    PointNormal::Ptr point_normal(new PointNormal);
    pcl::NormalEstimation<PointType, pcl::Normal> est_normal;
    est_normal.setInputCloud(in_cloud_ptr);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);
    //est_normal.setSearchSurface();
    est_normal.compute(*point_normal);

    //fpfh估计
    FpfhFeaturePtr fpfh(new FpfhFeature);
    //pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_target_fpfh;
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(4);//指定4核计算

    est_fpfh.setInputCloud(in_cloud_ptr);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(10);
    est_fpfh.compute(*fpfh);

    return fpfh;
}

float WalkTest::calculateSimilarity2(ClusterPtr cluster, CloudPtr model_cloud)
{
    CloudPtr cluster_cloud(new Cloud);
    CloudPtr filtered_cluster_cloud(new Cloud);
    CloudPtr filtered_model_cloud(new Cloud);
    CloudPtr final_cloud(new Cloud);
    transformPointCloud(cluster->getCloud(), cluster_cloud);
    // pcl::io::savePCDFile("x.pcd",*cluster_cloud);
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(model_cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*filtered_model_cloud);
    sor.setInputCloud(cluster_cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*filtered_cluster_cloud); // 此可否省去。因为点数本来就少

    KdtreePtr tree;
    FpfhFeaturePtr model_fpfh = computeFpfhFeature(filtered_model_cloud, tree);
    FpfhFeaturePtr cluster_fpfh = computeFpfhFeature(filtered_cluster_cloud, tree);
    pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputTarget(filtered_model_cloud);
    sac_ia.setTargetFeatures(model_fpfh);
    sac_ia.setInputSource(filtered_cluster_cloud);
    sac_ia.setSourceFeatures(cluster_fpfh);
    sac_ia.setNumberOfSamples(10);
    sac_ia.setCorrespondenceRandomness(6);
    sac_ia.setEuclideanFitnessEpsilon(0.001);
    sac_ia.setTransformationEpsilon(1e-4);
    sac_ia.setRANSACIterations(20);
    sac_ia.align(*final_cloud);
    float score = sac_ia.getFitnessScore();
    cout <<score << "okd" << endl;
    return score;
}

void
WalkTest::detectTarget(const CloudConstPtr &in_cloud_ptr, ClusterPtr target_cluster)//, ClusterPtr likelihood_target)
{
    int target_id = 100; // 避免下面的比较id时一致，设置一个类似越界的id，后考虑优化
    vector<pcl::PointIndices> cluster_indices;
    vector<ClusterPtr> clusters;
    vector<ClusterPtr> likelihood_clusters;
    CloudPtr removed_points_cloud(new Cloud);
    CloudPtr clipped_cloud(new Cloud);
    CloudPtr onlyfloor_cloud(new Cloud);
    ClusterPtr empty_cluster(new Cluster());
    //ClusterPtr likelihood_target;
    if (remove_points_upto_ > 0)
    {
        removePointsUpto(in_cloud_ptr, removed_points_cloud, remove_points_upto_);
    }
    else
    {
        pcl::copyPointCloud(*in_cloud_ptr, *removed_points_cloud);
    }
    //downsampleCloud
    //denoise
    // 此處的應用場景無需做此處理，截止線已經刪除了部分點雲數據，在其他應用場景中若需用此處理也加上判斷是否爲空處理
    clipCloud(removed_points_cloud, clipped_cloud, direction_, clip_min_height_, clip_max_height_, clip_right_pos_, clip_left_pos_, clip_bottom_pos_); //考慮加入遠截止線嗎？
    if (!clipped_cloud->empty())
    {
        if (remove_ground_)
        {
            removeFloor(clipped_cloud, nofloor_cloud_, onlyfloor_cloud, 0.1, 0.1);
        }
        else
        {
            nofloor_cloud_ = clipped_cloud;
        }
        if (!nofloor_cloud_->empty())
        {
            if (use_region_growing_)
            {
                cluster_indices = regiongrowingSegmentation(nofloor_cloud_);
            }
            else
            {
                cluster_indices = euclideanCluster(nofloor_cloud_, cluster_size_min_, cluster_size_max_, cluster_tolerance_);
            }
            //differenceNormalSegmentation
            //cluster_indices < 1 时返回false 后再完善逻辑
//            getCloudFromCluster(nofloor_cloud_, cluster_indices, clusters);
//            for (int i = 0; i < clusters.size(); ++i)
//            {
//                float interval_distance = calculatePointDistance(pre_target_centroid_, clusters[i]->getCentroid());
//                if (cloudinROI(clusters[i], region_points_size_, region_x_, region_x_) && interval_distance > )
//                {
//                    cout << "ROI has Interferon..." << endl;
//                }
//
//                if (clusters[i] >  <)
//                {
//                    likelihood_clusters.push_back(clusters[i]);
//                }
//
//            }

//            if (likelihood_clusters.size() >= 1)
//            {
//
//            }
            cout << "nofloor_cloud_: " << nofloor_cloud_->points.size() << endl;
            if (cluster_indices.size() != 0)
            {
                getCloudFromCluster(nofloor_cloud_, cluster_indices, target_min_height_, target_max_height_,
                                    target_min_width_, target_max_width_, &clusters, &likelihood_clusters);
                if (likelihood_clusters.size() != 0) {
                    succed_detect_counter_++;
                    vector<float> feature;
                    float min_dist = 100; //考虑用系统语言定义的max代替
                    int min_id = 0;
                    for (int i = 0; i < likelihood_clusters.size(); ++i) {
                        feature = extractClusterFeature(likelihood_clusters[i]);
                        float similarity = calculateSimilarity(feature, model_feature_);
                        //float similarity2 = calculateSimilarity2(likelihood_clusters[i], model_cloud_);  //model_cloud后可先处理好，不每次都处理
                        cout << "Similarity: " << similarity << endl;
                        if (similarity < min_dist) {
                            min_dist = similarity;
                            min_id = i;
                        }
                    }
                    //likelihood_target = likelihood_clusters[min_id];
                    //return true;
                    succed_detect_ = true;
                    target_id = min_id;
                    target_cluster = likelihood_clusters[target_id];
                    //return likelihood_clusters[target_id];
                } else {
                    //succed_detect_ = false;
                    //return false;
                    succed_detect_ = false;
                    target_cluster = empty_cluster;
                    //return empty_cluster;
                }

                //check if ROI has interferon;
                cout << "clusters:" << clusters.size() << endl;
                for (int i = 0; i < clusters.size(); ++i) {
                    if (succed_target_) {
                        float interval_distance = calculatePointDistance(pre_target_centroid_,
                                                                         clusters[i]->getCentroid());
                        if (cloudinROI(clusters[i], region_points_size_, region_x_, region_x_) &&
                            interval_distance > search_radius_)//暂定大于搜索半径
                        {
                            //需要累计障碍物个数吗，需要一检测到障碍物就提示吗？
                            cout << "ROI has Interferon..." << endl;
                            zlog_warn(c, "行人检测过程中，检测区域内发现障碍物");
                            _displayinfo.obstacledetection = 1;

                        } else {
                            _displayinfo.obstacledetection = 0;
                        }
                    } else {
                        if (cloudinROI(clusters[i], region_points_size_, region_x_, region_x_) &&
                            clusters[i]->getId() != target_id) //此id比较可否用于飞机检测那边的障碍物检测处理
                        {
                            //在此考虑 likelihood_clusters.size() == 0 的情况；此时 target_id 为 默认的越界值，上述条件也包含了likelihood_clusters.size() == 0的情况
                            cout << "ROI has Interferon..." << endl;
                            zlog_warn(c, "行人检测过程中，检测区域内发现障碍物");
                            _displayinfo.obstacledetection = 1;
                        } else {
                            _displayinfo.obstacledetection = 0;
                        }
                    }
                }

//            if (getCloudFromCluster(nofloor_cloud_, cluster_indices, target_min_height_, target_max_height_, target_min_width_, target_max_width_, &likelihood_clusters))
//            {
//                //succed_detect_ = true;
//                succed_detect_counter_++;
//                vector<float> feature;
//                float min_dist = 100; //考虑用系统语言定义的max代替
//                int min_id = 0;
//                for (int i = 0; i < likelihood_clusters.size(); ++i)
//                {
//                    feature = extractClusterFeature(likelihood_clusters[i]);
//                    float similarity = calculateSimilarity(feature, model_feature_);
//                    //float similarity2 = calculateSimilarity2(likelihood_clusters[i], model_cloud_);  //model_cloud后可先处理好，不每次都处理
//                    cout << "Similarity: " << similarity << endl;
//                    if (similarity < min_dist)
//                    {
//                        min_dist = similarity;
//                        min_id = i;
//                    }
//                }
//                //likelihood_target = likelihood_clusters[min_id];
//                //return true;
//                succed_detect_ = true;
//                return likelihood_clusters[min_id];
//            }
//            else
//            {
//                //succed_detect_ = false;
//                //return false;
//                succed_detect_ = false;
//                return empty_cluster;
//            }
            } else {
                cout << "Clustering failue" << endl;
                zlog_warn(c, "聚类出现异常,Clustering failue！");
                succed_detect_ = false;
                target_cluster = empty_cluster;
            }
        }
        else
        {
            cout << "removefloor failue" << endl;
            zlog_warn(c,"过滤地面出现异常,removefloor failue！");
            succed_detect_ = false;
            target_cluster = empty_cluster;
            //return empty_cluster;
        }
    }
    else
    {
        cout << "Region has no point cloud" << endl;
        zlog_warn(c,"区域没有点云数据，Region has no point cloud");
        succed_detect_ = false;
        target_cluster = empty_cluster;
        //return empty_cluster;
    }
    //return likelihood_target; //
}

void WalkTest::checkROI(const CloudConstPtr &cloud) {
    boost::mutex::scoped_lock lock(mtx_);
    CloudPtr ROI_cloud(new Cloud);
    CloudPtr nofloor_cloud(new Cloud);
    CloudPtr onlyfloor_cloud(new Cloud);
    vector<pcl::PointIndices> cluster_indices;

    if (cloud_id_ > 10) {
        appointRegion(cloud, ROI_cloud, region_points_size_, region_x_, region_y_);
        removeFloor(ROI_cloud, nofloor_cloud, onlyfloor_cloud, 0.1, 0.1);
        cluster_indices = euclideanCluster(nofloor_cloud, cluster_size_min_, cluster_size_max_, cluster_tolerance_);
        if (cluster_indices.size() != 0) {
            cout << "ROI has Interferon...行人检测时，自检模式下检测到有障碍物！！\n" << endl;
            zlog_warn(c, "行人检测时，自检模式下检测到有障碍物！！\n");
            _selfCheckinfo.obstacle_flag = 1;
        } else {

            cout << "ROI has Interferon...行人检测时，自检模式下没有障碍物！！\n" << endl;
            zlog_warn(c, "行人检测时，自检模式下没有障碍物！！\n");
            _selfCheckinfo.obstacle_flag = 0;

        }
    }
}

void WalkTest::processCloud(const CloudConstPtr &cloud)
{
    boost::mutex::scoped_lock lock(mtx_);
    //cloud_id_++;
    if (cloud_id_ > 10 )//&& cloud_id_ % 1 == 0)
    {
        //float radius = 1.2; //后可考虑根据速度及抽帧数来确定
        drawed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        vector<int> detect_target_indices;
        vector<int> track_target_indices;
        vector<int> current_target_indices;
        ClusterPtr detect_target_cluster(new Cluster());//(new Cluster());//(new Cluster()); //
        float detect_target_distance, track_target_distance;
        Input_Num detect_cen, track_cen;
        POSINFO status_ud, status_lr;
        bool choose_detect = false;
        bool succed_find = false;
        //int priority = 0;
        if (prev_cloud_->empty())
        {
            pcl::copyPointCloud(*cloud, *prev_cloud_);
            pre_distance_ = 0;
            pre_time_ = prev_cloud_->header.stamp;
        }
        else
        {
            cout << "<< **********  " << endl;
            detectTarget(cloud, detect_target_cluster);
            if (succed_detect_ == true)//detectTarget(cloud, detect_target_cluster)) //此處是指針傳參成功了嗎
            {
                detect_target_indices = detect_target_cluster->getPointIndices();
                //int test = detect_target_indices.size();
                //cout << "****************test:" << test << endl;
                vector<float> detect_feature = extractClusterFeature(detect_target_cluster);
                vector<float> track_feature = extractClusterFeature(track_target_cluster_);
                float detect_similarity = calculateSimilarity(detect_feature, model_feature_);
                float track_similarity = calculateSimilarity(track_feature, model_feature_);
                float similarity_difference = track_similarity - detect_similarity;
                cout <<"*************************" << "detect: " << detect_similarity << "  " << "track: " << track_similarity << endl;

                zlog_info(c,"detect = %d,    track = %d",detect_similarity,track_similarity);

                if (pre_target_cloud_->empty())//pre_target_cloud_->empty()//pre_target_indices_.empty()
                {
                    pcl::copyPointCloud(*detect_target_cluster->getCloud(), *pre_target_cloud_);
                    //pre_target_indices_ = detect_target_indices;
                    pre_target_centroid_.x = detect_target_cluster->getCentroid().x;
                    pre_target_centroid_.y = detect_target_cluster->getCentroid().y;
                    pre_target_centroid_.z = detect_target_cluster->getCentroid().z;
                }

                //detect_flag_ = true;
                float moving_distance = fabs(pre_distance_ - detect_target_cluster->getCentroid().y);
                float time = cloud->header.stamp - pre_time_;
                float velocity = 1000000 * moving_distance / time;
                pre_time_ = cloud->header.stamp;
                pre_distance_ = detect_target_cluster->getCentroid().y;
                //origin_distance_ = detect_target_cluster->getCentroid().y;
                detect_cen = {detect_target_cluster->getCentroid().x, detect_target_cluster->getCentroid().y};
                float inteval_distance = calculatePointDistance(pre_target_centroid_, detect_target_cluster->getCentroid());
                cout << "interval distance: " << inteval_distance << endl;
                cout << "lost counter: " << lost_counter_ << " similarity_difference: " << similarity_difference << endl;
                cout << "width: " << track_target_cluster_->getWidth() <<" "<< "height: " << track_target_cluster_->getHeight();

                if  (lost_counter_ >= 20) //||inteval_distance < 0.3 || track_target_cluster_->getWidth() < target_min_width_ ||
                    //track_target_cluster_->getWidth() > target_max_width_  || track_target_cluster_->getHeight() < target_min_height_ || track_target_cluster_->getHeight() > target_max_height_)// || similarity_difference > 0.25
                {
                    detect_flag_ = true;
                    _displayinfo.detectflag = detect_flag_;
                    choose_detect = true;
                    //priority = 1;
                    detect_target_distance = detect_target_cluster->getCentroid().y;
                    //pre_target_cloud_->clear();
                    //pcl::copyPointCloud(*detect_target_cluster->getCloud(), *pre_target_cloud_);
                    pre_target_centroid_.x = detect_target_cluster->getCentroid().x;
                    pre_target_centroid_.y = detect_target_cluster->getCentroid().y;
                    pre_target_centroid_.z = detect_target_cluster->getCentroid().z;
                    succed_target_ = true;
                    lost_counter_ = 0;
                }
                else
                {
                    choose_detect = false;
                }
            }

            cout << "track search for target..." << endl;
            if (findTarget(nofloor_cloud_, pre_target_centroid_, search_radius_,
                           &track_target_indices)) //radius 移出做类成员变量
            {
                detect_flag_ = true;

                delay_count = 0;
                succed_find = true;
                lost_counter_ = 0;
                //priority = 2;
                cout << "track search succed..." << endl;
                cout << "detect: " <<"(" << pre_target_centroid_.x << " " << pre_target_centroid_.y << " " << pre_target_centroid_.z << ")" << endl;
                int cluster_id = 0;
                ClusterPtr cluster(new Cluster());
                cluster->setCloud(nofloor_cloud_, track_target_indices, cluster_id, 255, 0 ,0);
                track_target_cluster_ = cluster;
                track_target_distance = cluster->getCentroid().y;
                track_cen = {cluster->getCentroid().x, cluster->getCentroid().y};
                pre_target_centroid_.x = cluster->getCentroid().x;
                pre_target_centroid_.y = cluster->getCentroid().y;
                pre_target_centroid_.z = cluster->getCentroid().z;
                cout << "track: " << "(" << pre_target_centroid_.x << " " << pre_target_centroid_.y << " " << pre_target_centroid_.z << ")" << endl;
                succed_target_ = true;
            }
            else
            {
                delay_count++;
                if(delay_count>20)
                {
                    detect_flag_ = false;
                }

                succed_find = false;
                succed_target_ = false;
                lost_counter_++;
            }

            if (choose_detect)
            {
                cout << "choose detect..." << endl;
//                LOG__(LOGID_I, "Status: detect Feature: length: %f width: %f height: %f\n", detect_target_cluster->getLength(), detect_target_cluster->getWidth(), detect_target_cluster->getHeight());
                zlog_info(c,"Status: detect Feature: length: %f width: %f height: %f\n", detect_target_cluster->getLength(), detect_target_cluster->getWidth(), detect_target_cluster->getHeight());

                //current_target_indices = detect_target_indices;
                origin_distance_ = detect_target_distance;
                status_ud = Plane_Straight_line_UD(direction_, end_point0_, end_point1_, detect_cen);
                end_distance_ = status_ud.offset;
//                if (mild_point0_.y >= end_point0_.y || mild_point1_.y >= end_point0_.y)
//                {
//                    end_distance_ = status_ud.offset;
//                }
//                else
//                {
//                    end_distance_ = -status_ud.offset;
//                }
                status_lr = Plane_Straight_line_LR(mild_point0_, mild_point1_, detect_cen);
                position_ = status_lr.position;
                offset_ = status_lr.offset;
                setTargetColor(nofloor_cloud_, detect_target_indices, drawed_cloud_, 0, 255, 0);
            }
            else
            {
                if (succed_find)
                {
                    cout << "choose track..." << endl;
//                    LOG__(LOGID_I, "Status: track Feature: length: %f width: %f height: %f\n", track_target_cluster_->getLength(), track_target_cluster_->getWidth(), track_target_cluster_->getHeight());
                    zlog_info(c,"Status: track Feature: length: %f width: %f height: %f\n", track_target_cluster_->getLength(), track_target_cluster_->getWidth(), track_target_cluster_->getHeight());

                    //current_target_indices = track_target_indices;
                    origin_distance_ = track_target_distance;
                    status_ud = Plane_Straight_line_UD(direction_, end_point0_, end_point1_, track_cen);
                    end_distance_ = status_ud.offset;
//                    if (mild_point0_.y >= end_point0_.y || mild_point1_.y >= end_point0_.y)
//                    {
//                        end_distance_ = status_ud.offset;
//                    }
//                    else
//                    {
//                        end_distance_ = -status_ud.offset;
//                    }
                    status_lr = Plane_Straight_line_LR(mild_point0_, mild_point1_, track_cen);
                    position_ = status_lr.position;
                    offset_ = status_lr.offset;
                    setTargetColor(nofloor_cloud_, track_target_indices, drawed_cloud_, 255, 0, 0);
                }
                else
                {
                    //current_target_indices = pre_target_indices_;
                    end_distance_ = pre_end_distance_;
                    position_ = pre_position_;
                    offset_ = pre_offset_;
                }
            }
            //pre_target_indices_ = current_target_indices;
            pre_end_distance_ = end_distance_;
            pre_position_ = position_;
            pre_offset_ = offset_;
            pcl::copyPointCloud(*cloud, *prev_cloud_);
        }
    }
    else
    {
        printf(". ");
    }
}

//void WalkTest::rawCloud(const boost::shared_ptr<const Cloud> &cloud)
//{
//    //if (!cloud->empty())
//    {
//        cloud_id_++;
//        //cout << "ok..........." << endl;
//        pcl::copyPointCloud(*cloud, *src_cloud_);
//        //saveSrcCloud(cloud);
//        char fname[50];
//
//        pcl::console::TicToc time;
//
//        if (cloud_id_ % 1 == 0)
//        {
//            time.tic();
//            processCloud(cloud);
//            cout << "Process " << cloud_id_ << "th frame cost " << time.toc() << "ms" << endl;
//            cout << "***************distance:" << origin_distance_ << endl;
//            if (!succed_target_)
//            {
////                end_distance_ = pre_end_distance_;
////                position_ = pre_position_;
////                offset_ = pre_offset_;
//                sprintf(fname, "./%s/%04d.pcd", "lost", cloud_id_);
//                pcl::io::savePCDFile(fname, *cloud);
//            }
//        }
//
//        cout << "******************" << "distance: " << end_distance_ << "*****************" << endl;
//        _mtx.lock();
//        _READYTOSENDFLAG = 1;// fail时需再回0吗？
//        _displayinfo.craft = "WalkTest";
//        _displayinfo.distance = end_distance_; //fabs(distance_);
//        _displayinfo.speed = 0.0; //velocity_;
//        _displayinfo.position = position_; //position_; //后再上判断
//        _displayinfo.offset = offset_; //offset_;
//        _displayinfo.detectflag = detect_flag_;
//        _mtx.unlock();
//    }
//    //else
//    //{
//      //  _workstatus = 5;
//    //}
//}

void WalkTest::flowCloud(const CloudConstPtr cloud)
{
    cout << "sdk work succed: " << endl;
    cloud_id_++;
    //cout << "ok..........." << endl;
    pcl::copyPointCloud(*cloud, *src_cloud_);
    //saveSrcCloud(cloud);
    char fname[50];

    pcl::console::TicToc time;

    if (cloud_id_ % 1 == 0)
    {
        time.tic();
        if (check_mode_ == 0) // 0: self checking
        {
            std::cout << " 已经进入自检模式！！" << std::endl;
            zlog_debug(c, "已经进入自检模式！！");

            checkROI(cloud);
        } else {
            processCloud(cloud);
            cout << "Process " << cloud_id_ << "th frame cost " << time.toc() << "ms" << endl;
            cout << "***************distance:" << origin_distance_ << endl;
            if (!succed_target_) {
//                end_distance_ = pre_end_distance_;
//                position_ = pre_position_;
//                offset_ = pre_offset_;
                sprintf(fname, "./%s/%04d.pcd", "lost", cloud_id_);
                //pcl::io::savePCDFile(fname, *cloud);
            }
        }

    }

    cout << "******************" << "distance: " << end_distance_ << "*****************" << endl;
    _mtx.lock();
    _READYTOSENDFLAG = 1;// fail时需再回0吗？
    _displayinfo.craft = "WalkTest";
    _displayinfo.distance = end_distance_; //fabs(distance_);
    _displayinfo.speed = 0.0; //velocity_;
    _displayinfo.position = position_; //position_; //后再上判断
    _displayinfo.offset = offset_; //offset_;
    _displayinfo.detectflag = detect_flag_;
    _mtx.unlock();


}

void WalkTest::lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    printf("lidar: time %lf , points %d\n", timestamp , cld->points.size());
    //cout << "sdk work ............." << endl;
    CloudPtr cloud(new Cloud);
    if (!cld->empty())
    {
        cloud->width = cld->width;
        cloud->height = cld->height;
        cloud->resize(cld->points.size());
        for (int i = 0; i < cld->points.size(); ++i)
        {
            cloud->points[i].x = cld->points[i].x;
            cloud->points[i].y = cld->points[i].y;
            cloud->points[i].z = cld->points[i].z;
            cloud->points[i].intensity = cld->points[i].intensity;
        }
        //AircraftDetect *tempfunc = (AircraftDetect*);
        pThis_->flowCloud(cloud);
    }
    cout << "sdk ok.........." << endl;
//    else
//    {
//        _workstatus = 5;
//    }

}

void WalkTest::gpsCallback(int timestamp)
{
    struct timeval ts;
    gettimeofday(&ts, NULL);
    int gpsTimestamp = timestamp;   // sample 里 gpsTimestamp与pandar40pToSysTimeGap是全局变量
    double pandar40pToSysTimeGap =
            static_cast<double>(ts.tv_sec) + \
      (static_cast<double>(ts.tv_usec) / 1000000.0) - \
      static_cast<double>(timestamp);
    printf("gps: %d, gap: %f\n", timestamp, pandar40pToSysTimeGap);
}



void WalkTest::viewPointCloud() //參照github上相關例程優化顯示
{
    while (1)
    {
        if (_run_flag == false)
        {
            break;
        }
    }

//    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
//    while (!cloud_viewer_->wasStopped())
//    {
//        if (succed_target_)
//        {
//            if (!cloud_viewer_->updatePointCloud(drawed_cloud_))
//            {
//                cloud_viewer_->addPointCloud(drawed_cloud_);
//            }
//        }
//        else
//        {
//            if (src_cloud_)
//            {
//                handler_.setInputCloud(src_cloud_);
//                if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI>(src_cloud_, handler_))
//                {
//                    cloud_viewer_->addPointCloud<pcl::PointXYZI>(src_cloud_, handler_);
//                }
//            }
//        }
//        cloud_viewer_->spinOnce(1, true);
////        if (!interface_.isRunning())
////        {
////            cloud_viewer_->spin();
////        }
//        //boost::this_thread::sleep(boost::posix_time::microseconds(3000));
//        if (_run_flag == false)
//        {
//            break;
//        }
//    }
}




void WalkTest::run()
{
    cout << "walktest run" << endl;
    show_personOrAircarft = 1;

    //pdsk_.start();
    pandar40p_.Start();
    viewPointCloud();
    _run_flag == false;
    cout<<"walktest read finish"<<endl;
    //pdsk_.stop();
    pandar40p_.Stop();
}

WalkTest::~WalkTest()
{
    // TODO Auto-generated destructor stub
}