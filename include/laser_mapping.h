#pragma once

#include <string>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <deque>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include "imu_processing.h"
#include "pointcloud_preprocess.h"
#include "ikd-Tree/ikd_Tree.h"

namespace fast_lio {

class LaserMapping : public rclcpp::Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LaserMapping(const std::string& sParamsDir);
    LaserMapping() = delete;
    ~LaserMapping() {
        fast_lio::Timer::PrintAll();
        LOG(INFO) << "laser mapping deconstruct";
    }
    bool LoadParamsFromYAML(const std::string &yaml);

    // callbacks of lidar and imu
    void StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void PublishOdometry();

    // sync lidar with imu
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    bool SyncPackages();
    void FovSegment();
    void MapIncremental();
    void Run();

   private:


    // tools
    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);
    // void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);
    // void PrintState(const state_ikfom &s);

private:
    // ros
    ::rclcpp::Node::SharedPtr node_handler_;
    std::string lidar_topic_;
    std::string imu_topic_;

    // sensor deque
    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

    /// modules
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process
    std::shared_ptr<KD_TREE<PointType>> ikdtree_ = nullptr;       // ikdtree
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    bool extrinsic_est_en_ = true;
    float det_range_ = 150.0f;
    double cube_len_ = 0;
    double filter_size_map_min_ = 0;

    // sync sensor deque
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    bool lidar_pushed_ = false;
    double last_timestamp_imu_ = -1.0;

    // IEKF state
    state_ikfom state_point_;                          // ekf current state
    vect3 pos_lidar_;                                  // lidar position after eskf update
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool localmap_initialized_ = false;
    BoxPointType localmap_box_; 

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    common::VV4F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<bool> point_selected_surf_;           // selected points
    common::VV4F plane_coef_;                         // plane coeffs
    bool flg_EKF_inited_ = false;

    /// ros pub and sub stuffs
    ::rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    ::rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    ::rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    /// options
    bool time_sync_en_ = false;
    bool flg_box_inited_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    
    double first_lidar_time_ = 0.0;
    

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    
    int pcd_index_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;  // get lidar scan num in SyncPackages()， to get mean lidar scan time
    bool timediff_set_flg_ = false;
    int effect_feat_num_ = 0, frame_num_ = 0; // frame_num: successed frame in run
    

    /////////////////////////  debug show / save /////////////////////////////////////////////////////////
    bool run_in_offline_ = true;
    bool path_pub_en_ = true;
    bool scan_pub_en_ = false;
    bool dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    bool scan_effect_pub_en_ = false;
    bool pcd_save_en_ = false;
    bool runtime_pos_log_ = true;
    int pcd_save_interval_ = -1;
    bool path_save_en_ = false;
    std::string dataset_;

};

}  // namespace fast_lio