//
// Created by luffy7n on 18-11-14.
//

#include "AircraftDetect.h"
#include "GlobleData.h"
extern struct control_msg con_msg;
extern int _workstatus;
extern bool _run_flag;

AircraftDetect *AircraftDetect::pThis_ = NULL;

void AircraftDetect::setParameter()
{
    cloud_id_ = 0;
    //orign_distance_ = 0;
    end_distance_ = 150;
//    mild_x_ = 0.0;
//    endline_y_ = 0.0;
//    x_precision_ = 0.0;
//    y_precision_ = 0.0;
    cluster_size_min_ = 100;
    cluster_size_max_ = 750;
    remove_points_upto_ = 0.0;
    clip_min_height_ = -20;
    clip_max_height_ = 10;
    clip_right_pos_ = -22;
    clip_left_pos_ = 20;
    cluster_tolerance_ = 0.8;
    target_min_height_ = 1.5;
    target_max_height_ = 4.5;
    target_min_width_ = 3.0;
    target_max_width_ = 4.0;
    max_y_ = 100;
    mild_point0_ = {con_msg.mild_point0_x, con_msg.mild_point0_y};
    mild_point1_ = {con_msg.mild_point1_x, con_msg.mild_point1_y};
    end_point0_ = {con_msg.end_point0_x, con_msg.end_point0_y};
    end_point1_ = {con_msg.end_point1_x, con_msg.end_point1_y};
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
    model_feature_.push_back(4);// 机头区域几何特征
    model_feature_.push_back(4);
}

void AircraftDetect::initializeParameter()
{
    detect_flag_ = 0;
    src_cloud_.reset(new Cloud);
    prev_cloud_.reset(new Cloud);
    nofloor_cloud_.reset(new Cloud);
    nofloor_cloud2_.reset(new Cloud);
    remove_ground_ = true;
    use_region_growing_ = false;
    //nose_y_ = 90;
    sum_wing_span_ = 0;
    sum_engine_interval_ = 0;
    sum_detect_parameter_ = 0;
    succed_detect_head_counter_ = 0;
    succed_detect_head_ = false;
    succed_detect_side_ = false;
    cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    cloud_viewer_->addLine(mild_p0_, mild_p1_, "mildline");
    cloud_viewer_->addLine(mild_p0l_, mild_p1l_, "mildline1");
    cloud_viewer_->addLine(mild_p0r_, mild_p1r_, "mildline2");
    cloud_viewer_->addLine(end_p0_, end_p1_, "endline");
    cloud_viewer_->addLine(end_p0u_, end_p1u_, "endline1");
    cloud_viewer_->addLine(end_p0d_, end_p1d_, "endline2");
}

void  AircraftDetect::loadAircraftData(const char *type_file, const char *aircraft_data)
{
    FILE *fp = fopen(type_file, "r");
    int n = 0;
    while (!feof(fp))
    {
        fscanf(fp, "%s\n", type_dict_[n++]);
    }
    fclose(fp);
    ifstream fin(aircraft_data);
    float vec_point;
    vector<float> vec;
    string line;
    vec_dict_.clear();
    while (!fin.eof())
    {
        getline(fin, line);
        stringstream stringin(line);
        vec.clear();
        while (stringin >> vec_point)
        {
            vec.push_back(vec_point);
        }
        vec_dict_.push_back(vec);
    }
}

AircraftDetect::AircraftDetect(string &calib_file, string &pcap_file, const char *type_file, const char *aircraft_data,
                               pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
: calibration_file_(calib_file)
, pcap_file_(pcap_file)
, handler_(handler)
//, interface_(calibration_file_, pcap_file_),
, pandar40p_(ip_, port_, 10110, lidarCallback, gpsCallback, 13500, 0,  string("hesai40"))
{
    pThis_ = this;
    setParameter();
    loadAircraftData(type_file, aircraft_data);
    //cv::generateColors(colors_, 100);
    initializeParameter();
}

AircraftDetect::AircraftDetect(string &calib_file, const char *type_file, const char *aircraft_data,
                               string &ip, int port,
                               pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
: calibration_file_(calib_file)
, ip_(ip)
, port_(port)
, handler_(handler)
//, interface_(boost::asio::ip::address::from_string(ip_), port_, calibration_file_)
, pandar40p_(ip_, port_, 10110, lidarCallback, gpsCallback, 13500, 0,  string("hesai40"))
{
    pThis_ = this;
    setParameter();
    loadAircraftData(type_file, aircraft_data);
    //cv::generateColors(colors_, 100);
    initializeParameter();
}

vector<float> AircraftDetect::extractClusterFeature(ClusterPtr cluster)
{
    vector<float> feature;
    feature.clear();
    float height = cluster->getHeight();
    float width = cluster->getWidth();
    feature.push_back(height);
    feature.push_back(width);
    return feature;
}

char* AircraftDetect::recognizeType(std::vector<float> feature)
{
    float min_distance = calculateSimilarity(feature, vec_dict_[0]);
    int min_index = 0;
    for (int i = 1; i < vec_dict_.size()-1; ++i)
    {
        float sim_d = calculateSimilarity(feature, vec_dict_[i]);
        if (sim_d < min_distance)
        {
            min_distance = sim_d;
            min_index = i;
        }
    }
    return type_dict_[min_index];
}

ClusterPtr AircraftDetect::detectTarget(const CloudConstPtr &in_cloud_ptr)
{
    CloudPtr removed_points_cloud(new Cloud);
    CloudPtr clipped_cloud(new Cloud);
    CloudPtr onlyfloor_cloud(new Cloud);
    vector<pcl::PointIndices> cluster_indices;
    vector<ClusterPtr> likelihood_clusters;
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
    clipCloud(removed_points_cloud, clipped_cloud, direction_, clip_min_height_, clip_max_height_, clip_right_pos_, clip_left_pos_, clip_bottom_pos_);
    if (remove_ground_)
    {
        removeFloor(clipped_cloud, nofloor_cloud_, onlyfloor_cloud, 0.2, 0.1);
    }
    else
    {
        nofloor_cloud_ = clipped_cloud;
    }
    if (use_region_growing_)
    {
        cluster_indices = regiongrowingSegmentation(nofloor_cloud_);
    }
    else
    {
        cluster_indices = euclideanCluster(nofloor_cloud_, cluster_size_min_, cluster_size_max_, cluster_tolerance_);
    }
    //differenceNormalSegmentation
    if (getCloudFromCluster(nofloor_cloud_, cluster_indices, target_min_height_, target_max_height_, target_min_width_, target_max_width_, &likelihood_clusters))
    {
        //succed_detect_counter_++;
        vector<float> feature;
        float min_dist = 100; //考虑用系统语言定义的max代替
        int min_id = 0;
        for (int i = 0; i < likelihood_clusters.size(); ++i)
        {
            /*feature = extractClusterFeature(likelihood_clusters[i]);
            float similarity = calculateSimilarity(feature, model_feature_);
            //float similarity2 = calculateSimilarity2(likelihood_clusters[i], model_cloud_);  //model_cloud后可先处理好，不每次都处理
            cout << "Similarity: " << similarity << endl;
            if (similarity < min_dist)
            {
                min_dist = similarity;
                min_id = i;
            }*/
            if (fabs(likelihood_clusters[i]->getMaxPoint().y) > 30)
            {
                min_id = i;
            }
        }
        //likelihood_target = likelihood_clusters[min_id];
        //return true;
        succed_detect_head_counter_++;
        //cout << "succed_detect_counter: " << succed_detect_counter_ << endl;
        succed_detect_head_= true;
        return likelihood_clusters[min_id];
    }
    else
    {
        succed_detect_head_ = false;
    }

}

ClusterPtr AircraftDetect::detectAircraftSide(const CloudConstPtr &in_cloud_ptr)
{
    CloudPtr cloud(new Cloud);
    CloudPtr passthrough_cloud(new Cloud);
    CloudPtr onlyfloor_cloud(new  Cloud);
    vector<pcl::PointIndices> cluster_indices;
    vector<ClusterPtr> likelihood_clusters;
    //Cluster aircraft_side;
    string field_name = "y";
    pcl::copyPointCloud(*in_cloud_ptr, *cloud);
    passthroughCloud(cloud, passthrough_cloud, -150, -50, field_name, false);
    removeFloor(passthrough_cloud, nofloor_cloud2_, onlyfloor_cloud);
    cluster_indices = euclideanCluster(nofloor_cloud2_, 100, 2000, 0.8);
    if (getCloudFromCluster(nofloor_cloud2_, cluster_indices, &likelihood_clusters))
    {
        succed_detect_side_ = true;
        return likelihood_clusters[0];
    }
    else
    {
        succed_detect_side_ = false;
    }
}

bool AircraftDetect::detectAircraftParameter(const CloudPtr &in_cloud_ptr, float *wing_span, float *engine_distance)
{
    double min_radius = 0.9, max_radius = 1.0;
    PointType min_point = findMinValues(in_cloud_ptr);
    PointType max_point = findMaxValues(in_cloud_ptr);
    *wing_span = max_point.x - min_point.x;
    CloudPtr left_cloud(new Cloud);
    CloudPtr right_cloud(new Cloud);
    CloudPtr left_circle(new Cloud);
    CloudPtr right_circle(new Cloud);
    CloudPtr circle(new Cloud);
    std::string field_name = "x";
    passthroughCloud(in_cloud_ptr, left_cloud, (ncen_x_+2), max_point.x, field_name, false);
    passthroughCloud(in_cloud_ptr, right_cloud, min_point.x, (ncen_x_-2), field_name, false);
    Eigen::VectorXf left_circle_coefficients, right_circle_coefficients;
    if (detectCircle(left_cloud, min_radius, max_radius, &left_circle_coefficients, left_circle)
        && detectCircle(right_cloud, min_radius, max_radius, &right_circle_coefficients, right_circle))
    {
        *engine_distance = left_circle_coefficients(0) - right_circle_coefficients(0);
        return true;
    }

}


void AircraftDetect::processCloud(const CloudConstPtr &cloud)
{
    vector<float> feature;
    ClusterPtr front_aircraft_head(new Cluster());
    ClusterPtr side_aircraft_fuselage;//(new Cluster());
    //usleep(10000);
    //ClusterPtr side_aircraft_fuselage(new Cluster());
    if (prev_cloud_->empty())
    {
        pcl::copyPointCloud(*cloud, *prev_cloud_);
    }
    else
    {
        front_aircraft_head = detectTarget(cloud);
        if (succed_detect_head_)
        {
            detect_flag_ = 1;
            if (succed_detect_head_counter_ == 1)
            {
                aircraft_length_ = calculateLength(length_);
                cout << "aircraft length is: " << aircraft_length_ << endl;
            }
            float moving_distance = fabs(pre_distance_ - front_aircraft_head->getCentroid().y);//第一次的距離無效
            float time = cloud->header.stamp - pre_time_;
            //double velocity_ = 1000000 * moving_distance / time;
            pre_time_ = cloud->header.stamp;
            pre_distance_ = front_aircraft_head->getCentroid().y;
            float head_distance = front_aircraft_head->getCentroid().y;
            float nose_y = front_aircraft_head->getMaxPoint().y;
            orign_distance_ = head_distance;
            //target_distance_ = front_aircraft_head_->getMaxPoint().y; //使用Plane_Straight_line_up()
            //Input_Num nose_point();
            head_centroid_location_.push_back(head_distance);
            if (head_centroid_location_.size() == 5)
            {
                velocity_ = calculateVelocity(head_centroid_location_);
                /*if ()// v = 0 && end_distance = 0
                {
                    _workstatus = 4;
                }*/
                cout << "velocity: " << velocity_ << endl;
                vector<float>::iterator First_Ele = head_centroid_location_.begin();
                head_centroid_location_.erase(First_Ele);
            }
            Input_Num nose_coord = {front_aircraft_head->getCentroid().x, front_aircraft_head->getMaxPoint().y}; //front_aircraft_head->getCentroid().x该为对应的机鼻点或者就这样
            POSINFO status_ud = Plane_Straight_line_UD(end_point0_, end_point1_, nose_coord);
            end_distance_ = status_ud.offset;
            Input_Num head_cen = {front_aircraft_head->getCentroid().x, front_aircraft_head->getCentroid().y};//用centroid更穩定些
            POSINFO status_lr = Plane_Straight_line_LR(mild_point0_, mild_point1_, head_cen);
            position_ = status_lr.position;
            offset_ = status_lr.offset;
            front_aircraft_.reset(new Cloud);
            string field_name = "y";
            passthroughCloud(nofloor_cloud_, front_aircraft_, (nose_y+0.5-aircraft_length_-5), (nose_y+0.5), field_name, false); //35 可改位計算出的機身長度，nose_y_再分析穩定性
            if (fabs(nose_y) >= 50 && fabs(nose_y) <= 60)
            {
                float wing_span = 0;
                float engine_interval = 0;
                if (detectAircraftParameter(front_aircraft_, &wing_span, &engine_interval))
                {
                    sum_wing_span_ += wing_span;
                    sum_engine_interval_ += engine_interval;
                    sum_detect_parameter_++;
                    if (abs(nose_y) >= 50 && abs(nose_y) <= 55)
                    {
                        feature.clear();
                        wing_span_ = sum_wing_span_ / sum_detect_parameter_;
                        engine_interval_ = sum_engine_interval_ / sum_detect_parameter_;
                        cout << "wing_span : " << wing_span_ << "m" << endl;
                        cout << "engine_interval : " << engine_interval_ << "m" << endl;
                        feature.push_back(wing_span_);
                        feature.push_back(engine_interval_);
                        feature.push_back(aircraft_length_);
                        type_ = recognizeType(feature);
                        cout << "type: " << type_ << endl;
                    }
                }
            }
        }
        else
        {
            //ClusterPtr side_aircraft_fuselage(new Cluster());//(new Cluster());
            //side_aircraft_fuselage.reset();
            side_aircraft_fuselage = detectAircraftSide(cloud);
            cout << "************1" << endl;
            if (succed_detect_side_)
            {
                cout << "************2" << endl;
                detect_flag_ = 1;
                vector<int> aircraft_indices;
                type_ = "Aircraft";
                //find_target_.reset(new Cloud);
                //float aircraft_distance = side_aircraft_cluster_->getCentroid().y;
                //target_distance_ = aircraft_distance;
                float side_distance = side_aircraft_fuselage->getCentroid().y;
                Input_Num fuselage_cen = {side_aircraft_fuselage->getCentroid().x, side_distance};
                orign_distance_ = side_distance;
                POSINFO side_status = Plane_Straight_line_UD(end_point0_, end_point1_, fuselage_cen);
                end_distance_ = side_status.offset;
                PointType cluster_centre;
                cluster_centre.x = side_aircraft_fuselage->getCentroid().x;
                cluster_centre.y = side_aircraft_fuselage->getCentroid().y;
                cluster_centre.z = side_aircraft_fuselage->getCentroid().z;
                float radius = 20.0f; //bigger;
                findTarget(nofloor_cloud2_, cluster_centre, radius, &aircraft_indices);
                ClusterPtr cluster(new Cluster);
                cluster->setCloud(nofloor_cloud2_, aircraft_indices, 0, 255, 0, 0); // 函数可设0, 255, 0, 0默认值
                side_aircraft_.reset(new Cloud);
                side_aircraft_ = getCloudByIndices(nofloor_cloud2_, aircraft_indices);
                float aircraft_length = calculateSideLength(side_aircraft_);//sqrtf((cluster->getMaxPoint().x-cluster->getMinPoint().x)*(cluster->getMaxPoint().x-cluster->getMinPoint().x)+
                                                      //(cluster->getMaxPoint().y-cluster->getMinPoint().y)*(cluster->getMaxPoint().y-cluster->getMinPoint().y));
                cout << "aircraft_length: " << aircraft_length << endl;
                length_.push_back(aircraft_length);
            }
            else
            {
                cout << "*******lost target*******" << endl;
            }
        }
    }
    cout << "***********************3" << endl;
    cout << "******************orign distance: " << orign_distance_ << endl;
}

void AircraftDetect::flowCloud(const CloudConstPtr cloud)
{
    cloud_id_ ++;
    pcl::copyPointCloud(*cloud, *src_cloud_);
    //saveSrcCloud(cloud);
    pcl::console::TicToc time;
    if (cloud_id_ > 10 && cloud_id_ % 1 == 0)
    {
        time.tic();
        /*try {
                processCloud(cloud);
            }
           catch (exception& e) {
                cout << "Standard exception: " << e.what() << endl;
            }
                cout << "process " << cloud_id_ << "th frame cost " << time.toc() << "ms." << endl;*/
        processCloud(cloud);
        //cout << cloud_id_ << "frame***************" << endl;
        cout << "process " << cloud_id_ << "th frame cost " << time.toc() << "ms." << endl;
    }

    //sleep(1);
    _mtx.lock();
    _READYTOSENDFLAG = 1;
    _displayinfo.craft = type_;//pedestrian
    _displayinfo.distance = end_distance_; //(double)abs(distance_);
    _displayinfo.speed = velocity_; //velocity_;
    _displayinfo.position = position_; //position_;
    _displayinfo.offset = offset_; //offset_;
    _displayinfo.detectflag = detect_flag_;
    _mtx.unlock();
}

void AircraftDetect::lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
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

void AircraftDetect::gpsCallback(int timestamp)
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


//void AircraftDetect::rawCloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> > &cloud)
//{
//    //this->viewer_.showCloud(cloud);
//    if (!cloud->empty())
//    {
//        cloud_id_ ++;
//        pcl::copyPointCloud(*cloud, *src_cloud_);
//        //saveSrcCloud(cloud);
//        pcl::console::TicToc time;
//        if (cloud_id_ > 10 && cloud_id_ % 1 == 0)
//        {
//            time.tic();
//            /*try {
//                processCloud(cloud);
//            }
//            catch (exception& e) {
//                cout << "Standard exception: " << e.what() << endl;
//            }
//                cout << "process " << cloud_id_ << "th frame cost " << time.toc() << "ms." << endl;*/
//            processCloud(cloud);
//            //cout << cloud_id_ << "frame***************" << endl;
//            cout << "process " << cloud_id_ << "th frame cost " << time.toc() << "ms." << endl;
//        }
//
//        //sleep(1);
//        _mtx.lock();
//        _READYTOSENDFLAG = 1;
//        _displayinfo.craft = type_;//pedestrian
//        _displayinfo.distance = -end_distance_; //(double)abs(distance_);
//        _displayinfo.speed = velocity_; //velocity_;
//        _displayinfo.position = position_; //position_;
//        _displayinfo.offset = offset_; //offset_;
//        _displayinfo.detectflag = detect_flag_;
//        _mtx.unlock();
//    }
//    else
//    {
//        _workstatus = 5;
//    }
//
//}

void AircraftDetect::viewPointCloud()
{
    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
    while (!cloud_viewer_->wasStopped())
    {
        if (front_aircraft_)//succed_detect_head_
        {
            handler_.setInputCloud(front_aircraft_);
            if (!cloud_viewer_->updatePointCloud(front_aircraft_, handler_))
            {
                cloud_viewer_->addPointCloud(front_aircraft_, handler_);
            }
        }
        else
        {
            if (side_aircraft_)//succed_detect_side_
            {
                handler_.setInputCloud(side_aircraft_);
                if (!cloud_viewer_->updatePointCloud(side_aircraft_, handler_))
                {
                    cloud_viewer_->addPointCloud(side_aircraft_, handler_);
                }
            }
            else
            {
                if (src_cloud_)
                {
                    handler_.setInputCloud(src_cloud_);
                    if (!cloud_viewer_->updatePointCloud(src_cloud_, handler_))
                    {
                        cloud_viewer_->addPointCloud(src_cloud_, handler_);
                    }
                }
            }
        }
        cloud_viewer_->spinOnce(1, true);
//        if (!interface_.isRunning()) // 此段不需要吗？
//        {
//            cloud_viewer_->spin();
//        }
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
        if (_run_flag == false)
        {
            break;
        }
    }
}

//void AircraftDetect::run()
//{
//    boost::function<void(const boost::shared_ptr<const Cloud> &)> f =
//            boost::bind(&AircraftDetect::rawCloud, this, _1);
//    boost::signals2::connection c = interface_.registerCallback(f);
//    interface_.start();
//    //controlKey(cloud_id_);
//    viewPointCloud();
//    _run_flag == false;
//    cout<<"AircraftDetect read finish"<<endl;
//    interface_.stop();
//    c.disconnect();
//}

void AircraftDetect::run()
{
    //pdsk_.start();
    pandar40p_.Start();
    viewPointCloud();
    _run_flag == false;
    cout<<"walktest read finish"<<endl;
    //pdsk_.stop();
    pandar40p_.Stop();
}

AircraftDetect::~AircraftDetect()
{
    // TODO Auto-generated destructor stub
}