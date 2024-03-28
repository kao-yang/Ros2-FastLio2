#include "laser_mapping.h"

namespace fast_lio {

LaserMapping::LaserMapping(const std::string& sParamsDir)
        : Node("ros2_fast_lio2")
{
    // Step 0. init ros2 node handle
    node_handler_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    // Step 1.init ptr


    // Step 2.load params
    if ( !LoadParamsFromYAML( sParamsDir+ "ikdodom.yaml") ){
        LOG(FATAL) << "load param fail,process exit";
    }else{
        LOG(INFO) << "load param success";
    }

    m_slam = std::make_unique<::humanoid_slam::HumanoidSlamBuilder>
                            ( sParamsDir + "humanoid_slam.yaml" );

    // Step 3. sub
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 1000, std::bind(&LaserMapping::IMUCallBack, this, std::placeholders::_1));
    sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, 100, std::bind(&LaserMapping::StandardPCLCallBack, this, std::placeholders::_1));
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);
    try {
        lidar_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    return true;
}



void LaserMapping::StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    mtx_buffer_.lock();
    auto cloudData = RsHoliesToTimedPointCloudData(msg);
    m_slam->AddSensorData("/lidar", cloudData);
    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg){
    mtx_buffer_.lock();
    auto imuData = ToImuData(msg);
    m_slam->AddSensorData("/imu", imuData);
    mtx_buffer_.unlock();
    
}


::humanoid_slam::sensor::TimedPointCloudData
        LaserMapping::RsHoliesToTimedPointCloudData
        ( const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pMsg ){
    // Step 1. get time
    double fEndTime = FromRosTime( pMsg->header.stamp );

    // Step 2. get origin cloud
    pcl::PointCloud<rsHoliesRos::Point> originPointCloud;
    pcl::fromROSMsg( *pMsg, originPointCloud );
    double fBeginTime = originPointCloud.points[0].timestamp;
    CHECK_GT(fBeginTime, 0);
    CHECK_GT(fEndTime, fBeginTime);

    // Step 3. filter blind, get aimed cloud
    size_t nSize = originPointCloud.points.size();
    CloudPtr pCloudOut( new PointCloudType );
    pCloudOut->points.reserve( nSize );
    size_t i = 0;
    for( auto &it : originPointCloud.points ){
        // Step 3.1 transform data type
        PointType pointOut;
        pointOut.normal_x = 0;
        pointOut.normal_y = 0;
        pointOut.normal_z = 0;
        pointOut.x = it.x;
        pointOut.y = it.y;
        pointOut.z = it.z;
        pointOut.intensity = it.intensity;
        pointOut.curvature = (it.timestamp - fBeginTime)*1000.f;  // point time in wall time, unit:s
    
        // Step 3.2 downsampling cloud points
        if ( i % 1 == 0 ){
            // Step 3.3 filter blind
            double fLen = sqrt( pointOut.x * pointOut.x + pointOut.y * pointOut.y + pointOut.z * pointOut.z );
            if ( fLen >= 1.0 && fLen <= 100.0 ){
                pCloudOut->points.emplace_back( pointOut );
            }
        }
        i++;
    }
    return ::humanoid_slam::sensor::TimedPointCloudData{ fBeginTime, fEndTime, pCloudOut };
}

::humanoid_slam::sensor::ImuData 
        LaserMapping::ToImuData( const sensor_msgs::msg::Imu::ConstSharedPtr& pMsg ){
    double fTime = FromRosTime( pMsg->header.stamp );
    Eigen::Vector3d angVec = Eigen::Vector3d(pMsg->angular_velocity.x, pMsg->angular_velocity.y, 
                                pMsg->angular_velocity.z);
    Eigen::Vector3d linAcc = Eigen::Vector3d(pMsg->linear_acceleration.x, pMsg->linear_acceleration.y, 
                                pMsg->linear_acceleration.z);
    return ::humanoid_slam::sensor::ImuData{fTime, angVec, linAcc};
}

} // namespace fast_lio