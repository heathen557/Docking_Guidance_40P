//
// Created by luffy7n on 18-11-14.
//

#ifndef DOCKING_GUIDANCE2_AIRCRAFTDETECT_H
#define DOCKING_GUIDANCE2_AIRCRAFTDETECT_H

//#include "pandar_grabber/pandar_grabber.h"
#include "pandar40p_sdk/pandar40p_sdk.h"
#include "ProcessPointcloud.h"

extern int show_personOrAircarft;
extern boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

extern CloudPtr src_cloud_;
extern CloudPtr front_aircraft_;
extern CloudPtr side_aircraft_;

class AircraftDetect
{
    bool angleGreater45_flag;
    bool succed_track_;
    bool start_track_;
    bool succed_target_;
    PointType pre_target_centroid_;
    float head_radius_;
    ClusterPtr track_target_cluster_;   //跟踪到飞机的点云数据
    bool succed_detect_front_head_;     //是否检测到机头的标识
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pre_target_cloud_;    //前面跟踪到的机头的点云数据
    float nose_passenger_door_;



    int line_count;
    int port_,cloud_id_;
    int detect_flag_;
    int cluster_size_min_, cluster_size_max_;
    float remove_points_upto_, clip_min_height_, clip_max_height_, clip_left_pos_, clip_right_pos_, clip_bottom_pos_, cluster_tolerance_,
            target_min_height_, target_max_height_, target_min_width_, target_max_width_, max_y_;
    float nose_y_, ncen_x_, ncen_y_;
    float sum_wing_span_, sum_engine_interval_;
    int sum_detect_parameter_;
    int succed_detect_head_counter_;
    float wing_span_, engine_interval_, aircraft_length_;
    float pre_time_, pre_distance_;
    float orign_distance_, end_distance_, offset_, velocity_;
    bool direction_;
    bool remove_ground_;
    bool use_region_growing_;
    bool succed_detect_head_;
    bool succed_detect_side_;
    bool succed_find_;
    char type_dict_[20][15];
    string position_, type_, calibration_file_, pcap_file_, ip_;
    Input_Num mild_point0_, mild_point1_, end_point0_, end_point1_;
    vector<float> model_feature_;
    vector<float> length_;
    vector<float> head_centroid_location_;
    vector<vector<float>> vec_dict_;

    pcl::PointXYZ mild_p0_, mild_p1_, end_p0_, end_p1_,  mild_p0l_, mild_p1l_, end_p0u_, end_p1u_,  mild_p0r_, mild_p1r_, end_p0d_, end_p1d_;//后考虑如何自动确定中心线;
//    CloudPtr src_cloud_;                          //note 2019-3-29
    CloudPtr prev_cloud_;
    CloudPtr nofloor_cloud_, nofloor_cloud2_;
//    CloudPtr front_aircraft_, side_aircraft_;     //note 2019-3-29
    CloudPtr find_target_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud_;
    vector<ClusterPtr> target_;
    vector<ClusterPtr> aircraft_;
    //pcl::visualization::CloudViewer viewer_;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
//    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
    //pcl::PandarGrabber interface_;
    Pandar40PSDK pandar40p_;
    static AircraftDetect *pThis_;

public:

    void loadAircraftData(const char *type_file, const char *aircraft_data);
    virtual ~AircraftDetect();
    vector<float> extractClusterFeature(ClusterPtr cluster);
    ClusterPtr detectTarget(const CloudConstPtr &in_cloud_ptr);
    bool detectAircraftParameter(const CloudPtr &in_cloud_ptr, float *wing_span, float *engine_distance);
    char *recognizeType(std::vector<float> feature);
    void processCloud(const CloudConstPtr &cloud);
    void rawCloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud);
    void viewPointCloud();



    //new 2019-03-27
    AircraftDetect(string &calib_file, string &pcap_file, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    AircraftDetect(string &calib_file, string &ip, int port, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    static void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp);
    static void gpsCallback(int timestamp);
    void setParameter();
    void initializeParameter();
    void run();
    void flowCloud(const CloudConstPtr cloud);
    void calculate_angle(const CloudConstPtr &cloud);
    void calculate_aircraftLength(const CloudConstPtr &cloud);
    ClusterPtr detectAircraftSide(const CloudConstPtr &in_cloud_ptr);
    void target_detectionAndTrack(const CloudConstPtr &cloud);
    bool preprocessCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr);

    void targetToProcess(vector<int> currentTarget_indices, ClusterPtr aircraftFrontHeadCluster);

    void localfileTest();










};


#endif //DOCKING_GUIDANCE2_AIRCRAFTDETECT_H
