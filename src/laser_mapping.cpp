#include "laser_mapping.h"

namespace fast_lio {

LaserMapping::LaserMapping(const std::string& sParamsDir)
        : Node("ros2_fast_lio2")
{
        // Step 0. init ros2 node handle
    node_handler_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    // Step 1.init ptr
    // TODO

    // Step 2.load params
    if ( !LoadParamsFromYAML( sParamsDir+ "ikdodom.yaml") ){
        LOG(FATAL) << "load param fail,process exit";
    }else{
        LOG(INFO) << "load param success";
    }

    // Step 3. sub
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, std::bind(&LaserMapping::IMUCallBack, this, std::placeholders::_1));
    sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, 10, std::bind(&LaserMapping::StandardPCLCallBack, this, std::placeholders::_1));

}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    // common::V3D lidar_T_wrt_IMU;
    // common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        // // 常规参数
        // time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();
        // mbLoadMapAndLoop = yaml["common"]["load_map_and_loop"].as<bool>();
        // mbUpdateMapAndLoop = yaml["common"]["update_map_and_loop"].as<bool>();
        lidar_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        // // 雷达预处理参数
        // lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        // preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        // preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        // preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        // preprocess_->PointFilterNum() = yaml["preprocess"]["point_filter_num"].as<int>();
        // // 建图相关参数
        // cube_len_ = yaml["mapping"]["cube_side_length"].as<double>();
        // filter_size_surf_min = yaml["mapping"]["filter_size_surf"].as<double>();
        // filter_size_map_min_ = yaml["mapping"]["filter_size_map"].as<float>();
        options::NUM_MAX_ITERATIONS = yaml["mapping"]["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["mapping"]["esti_plane_threshold"].as<float>();
        // acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        // gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        // b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        // b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        // det_range_ = yaml["mapping"]["det_range"].as<float>();
        // extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        // extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        // extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
        // // cloud转scan地图相关参数
        // p2s_->ThGroundDeg() = yaml["pmap"]["th_ground_deg"].as<float>();
        // p2s_->ThZBot() = yaml["pmap"]["th_z_bot"].as<float>();
        // p2s_->ThZTop() = yaml["pmap"]["th_z_top"].as<float>();
        // p2s_->ThRangeMin() = yaml["pmap"]["th_range_min"].as<float>();
        // p2s_->ThRangeMax() = yaml["pmap"]["th_range_max"].as<float>();
        // p2s_->ThIntensity() = yaml["pmap"]["th_intensity"].as<float>();
        // p2s_->NumberScan() = yaml["pmap"]["num_scan"].as<int>();
        // p2s_->HorizonScan() = yaml["pmap"]["horizon_scan"].as<int>();
        // p2s_->AngResY() = yaml["pmap"]["ang_res_y"].as<float>();
        // pm_->MapResolution() = yaml["pmap"]["map_resolution"].as<float>();
        // // 发布相关参数
        // path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        // scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        // scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        // dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        // scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        // // 保存path使能
        // path_save_en_ = yaml["path_save_en"].as<bool>();
        // // 保存pcd相关参数
        // pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        // pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        // 其它参数???
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    // LOG(INFO) << "lidar_type " << lidar_type;
    // if (lidar_type == 1) {
    //     preprocess_->SetLidarType(LidarType::VELO16);
    //     LOG(INFO) << "Using Velodyne 16 Lidar";
    // } else if (lidar_type == 2) {
    //     preprocess_->SetLidarType(LidarType::OUST64);
    //     LOG(INFO) << "Using OUST 64 Lidar";
    // } else if (lidar_type == 3){
    //     preprocess_->SetLidarType(LidarType::RS16);
    //     LOG(INFO) << "Using RS 16 Lidar";
    // }else {
    //     LOG(WARNING) << "unknown lidar_type";
    //     return false;
    // }

    // voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // voxel_map_.setLeafSize(0.1, 0.1, 0.1);

    // lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    // lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    // p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    // p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    // p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    // p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    // p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // LOG(WARNING) << "extrin T: " << lidar_T_wrt_IMU.transpose();
    // LOG(WARNING) << "extrin R: " << lidar_R_wrt_IMU.transpose();

    return true;
}



void LaserMapping::StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    mtx_buffer_.lock();
    // Timer::Evaluate(
    //     [&, this]() {
    //         PointCloudType::Ptr ptr(new PointCloudType());
    //         PointCloudType::Ptr wout_ground(new PointCloudType());

    //         sensor_msgs::PointCloud2::Ptr changeMsg (new sensor_msgs::PointCloud2(*msg));
    //         // timestamp correct for rs16
    //         if (preprocess_->GetLidarType()==LidarType::RS16){
    //             double timeOffSet = 0;
    //             unsigned char tmp[8];
    //             for(int i=0;i<8;i++){
    //                 tmp[i] = changeMsg->data[18+i];
    //             }
    //             timeOffSet = *(double*)tmp;
    //             // LOG(WARNING) << "timeOffSet:" << std::to_string(timeOffSet);
    //             // LOG(WARNING) << "time-0.1:" << std::to_string(changeMsg->header.stamp.toSec()-0.1);
    //             changeMsg->header.stamp.fromSec(timeOffSet);
    //         }
            

    //         preprocess_->Process(changeMsg, ptr);
    //         lidar_buffer_.push_back(ptr);
    //         time_buffer_.push_back(changeMsg->header.stamp.toSec());
    //         while (lidar_buffer_.size()>100){
    //             lidar_buffer_.pop_front();
    //         }
    //         while (time_buffer_.size()>100){
    //             time_buffer_.pop_front();
    //         }
    //         last_timestamp_lidar_ = changeMsg->header.stamp.toSec();
    //     },
    //     "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg){
    mtx_buffer_.lock();
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

} // namespace fast_lio