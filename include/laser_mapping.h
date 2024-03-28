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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "humanoid_slam/sensor/ImuData.h"
#include "humanoid_slam/sensor/TimedPointCloudData.h"
#include "humanoid_slam/sensor/LidarOdomMeasureData.h"
#include "humanoid_slam/HumanoidSlamInterface.h"
#include "humanoid_slam/HumanoidSlamBuilder.h"
#include "sophus/se3.hpp"


namespace rsHoliesRos {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(rsHoliesRos::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (std::uint16_t, ring, ring)(double, timestamp, timestamp))

namespace fast_lio {

class LaserMapping : public rclcpp::Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LaserMapping(const std::string& sParamsDir);
    LaserMapping() = delete;
    ~LaserMapping() {
        LOG(INFO) << "laser mapping deconstruct";
    }
    bool LoadParamsFromYAML(const std::string &yaml);

    // callbacks of lidar and imu
    void StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

private:
    double FromRosTime(const builtin_interfaces::msg::Time& time){
        return time.sec + static_cast<double>(time.nanosec) / 1000000000.f;
    }


private:
    // ros
    ::rclcpp::Node::SharedPtr node_handler_;
    std::string lidar_topic_;
    std::string imu_topic_;

    // sensor deque
    std::mutex mtx_buffer_;

    /// ros pub and sub stuffs
    ::rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    ::rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    ::rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<::humanoid_slam::HumanoidSlamInterface> m_slam;
    

public:
    ::humanoid_slam::sensor::TimedPointCloudData
        RsHoliesToTimedPointCloudData
        ( const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pMsg );

    ::humanoid_slam::sensor::ImuData 
        ToImuData( const sensor_msgs::msg::Imu::ConstSharedPtr& pMsg );
};

}  // namespace fast_lio