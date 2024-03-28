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


    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    
    
    

    // sync sensor deque
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    
    

    // IEKF state
    
    
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool localmap_initialized_ = false;
     

    /// point clouds data
    
    
    
    
    
    
    
    
    

    /// ros pub and sub stuffs
    ::rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    ::rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    ::rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    /// options
    
    
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    
    int frame_num_ = 0;
    

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    
    
    int pcd_index_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;  // get lidar scan num in SyncPackages()， to get mean lidar scan time
    bool timediff_set_flg_ = false;
    
    

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

///////////////////////////////////////////
    double m_fCubeLen = 0;
    double m_fFilterSizeMapMin = 0;
    float m_fLidarDetectRange = 150.0f;
    bool m_bExtrinsicEstEn = true;
    bool m_bTimeSyncEn = false;

    pcl::VoxelGrid<PointType> m_voxelScan;            // voxel filter for current scan

    bool m_bLidarPushed = false;
    double m_fLastImuTime = -1.0;

    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr m_pCloudDsInLidarBodyFrame{new PointCloudType()};   // downsampled scan in body
    CloudPtr m_pCloudDsInMapFrame{new PointCloudType()};  // downsampled scan in world

    bool m_bFirstScan = true;
    bool m_bBbxInited = false;
    bool m_bEkfInited = false;
    double m_fFirstLidarTime = 0.0;
    BoxPointType m_localMapBbx;
    std::vector<PointVector> m_vNearestPoints;         // nearest points of current scan
    std::vector<bool> m_vPointSelectedSurf;           // selected points
    common::VV4F m_planeCoef;                         // plane coeffs
    std::vector<float> m_vResiduals;                    // point-to-plane residuals
    size_t m_nEffectFeatureNum = 0;  // frame_num: successed frame in run
    common::VV4F m_effectFeaturePoints;                           // inlier pts
    common::VV4F m_effectFeatureNormals;                          // inlier plane norms
    state_ikfom m_ikfState;                          // ekf current state
    vect3 m_lidarPoseInMapFrame;                                  // lidar position after eskf update

    std::shared_ptr<ImuProcess> m_pImu = nullptr;                 // imu process
    std::shared_ptr<KD_TREE<PointType>> m_pIkdTree = nullptr;       // ikdtree
    esekfom::esekf<state_ikfom, 12, input_ikfom> m_kf;  // esekf

};

}  // namespace fast_lio