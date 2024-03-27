#include "laser_mapping.h"

namespace fast_lio {

LaserMapping::LaserMapping(const std::string& sParamsDir)
        : Node("ros2_fast_lio2")
{
    // Step 0. init ros2 node handle
    node_handler_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    // Step 1.init ptr
    preprocess_.reset( new PointCloudPreprocess() );
    p_imu_.reset( new ImuProcess() );
    ikdtree_.reset( new KD_TREE<PointType>() );

    // Step 2.load params
    if ( !LoadParamsFromYAML( sParamsDir+ "ikdodom.yaml") ){
        LOG(FATAL) << "load param fail,process exit";
    }else{
        LOG(INFO) << "load param success";
    }

    // Step 3. init iekf
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        // 等价于fastlio的h_share_model
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

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
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        // // 常规参数
        // time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();
        // mbLoadMapAndLoop = yaml["common"]["load_map_and_loop"].as<bool>();
        // mbUpdateMapAndLoop = yaml["common"]["update_map_and_loop"].as<bool>();
        lidar_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        // // 雷达预处理参数
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        preprocess_->PointFilterNum() = yaml["preprocess"]["point_filter_num"].as<int>();
        // // 建图相关参数
        cube_len_ = yaml["mapping"]["cube_side_length"].as<double>();
        filter_size_surf_min = yaml["mapping"]["filter_size_surf"].as<double>();
        filter_size_map_min_ = yaml["mapping"]["filter_size_map"].as<float>();
        options::NUM_MAX_ITERATIONS = yaml["mapping"]["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["mapping"]["esti_plane_threshold"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
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

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::VELO16);
        LOG(INFO) << "Using Velodyne 16 Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 3){
        preprocess_->SetLidarType(LidarType::RS16);
        LOG(INFO) << "Using RS 16 Lidar";
    }else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    // voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // voxel_map_.setLeafSize(0.1, 0.1, 0.1);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    LOG(WARNING) << "extrin T: " << lidar_T_wrt_IMU.transpose();
    LOG(WARNING) << "extrin R: " << lidar_R_wrt_IMU.transpose();

    return true;
}



void LaserMapping::StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            PointCloudType::Ptr ptr(new PointCloudType());

            sensor_msgs::msg::PointCloud2::SharedPtr changeMsg (new sensor_msgs::msg::PointCloud2(*msg));
            // timestamp correct for rs16
            if (preprocess_->GetLidarType()==LidarType::RS16){
                double timeOffSet = 0;
                unsigned char tmp[8];
                for(int i=0;i<8;i++){
                    tmp[i] = changeMsg->data[18+i];
                }
                timeOffSet = *(double*)tmp;
                changeMsg->header.stamp = fast_lio::common::ToRosTime(timeOffSet);
            }
            

            preprocess_->Process(changeMsg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back( fast_lio::common::FromRosTime(changeMsg->header.stamp) );
            // LOG(INFO) << "time buffer input, time: " << std::to_string( fast_lio::common::FromRosTime(changeMsg->header.stamp) );
            // LOG(INFO) << "lidar buffer input, size: " << ptr->points.size();

            last_timestamp_lidar_ = fast_lio::common::FromRosTime(changeMsg->header.stamp);
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr msg){
    mtx_buffer_.lock();
    auto timestamp = common::FromRosTime(msg->header.stamp);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
    Run();
}


bool LaserMapping::SyncPackages(){
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** get new lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;

        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = common::FromRosTime( imu_buffer_.front()->header.stamp );
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = common::FromRosTime( imu_buffer_.front()->header.stamp );
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::FovSegment(){
    // step1.声明待删除的地图box队列:cub_needrm
    vector<BoxPointType> cub_needrm;
    vector<BoxPointType> cub_needadd;
    cub_needrm.clear();
    // step2.获得lidar在w系下的位姿
    vect3 pos_lid = pos_lidar_;
    // step3.初始化box
    if(!flg_box_inited_){
        for (int i = 0; i < 3; i++){
            localmap_box_.vertex_min[i] = pos_lid(i) - cube_len_ / 2.0;
            localmap_box_.vertex_max[i] = pos_lid(i) + cube_len_ / 2.0;
        }
        flg_box_inited_ = true;
        return;
    }
    // step4. 求取距离和是否需要移动
    float dist_to_map_edge[3][2];   // lidar与立方体六个面的距离
    bool need_move = false;         // 是否需要移动
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_lid(i) - localmap_box_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lid(i) - localmap_box_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * det_range_ 
           || dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * det_range_)
            need_move = true;
    }
    if (!need_move) return;

    // step5. 进行移动localmap操作
    BoxPointType New_LocalMap_Points, tmp_boxpoints_rm, tmp_boxpoints_add;  // 新的局部地图盒子边界点
    New_LocalMap_Points = localmap_box_;
    // step5.1 求取移动距离
    float mov_dist = max((cube_len_ - 2.0 * options::MOV_THRESHOLD * det_range_) * 0.5 * 0.9, 
                          double(det_range_ * (options::MOV_THRESHOLD -1)));
    // step5.2 求取待删除的矩形大小
    for (int i = 0; i < 3; i++){
        tmp_boxpoints_rm = localmap_box_;
        tmp_boxpoints_add = localmap_box_;
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * det_range_){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints_rm.vertex_min[i] = localmap_box_.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints_rm);
            tmp_boxpoints_add.vertex_min[i] = New_LocalMap_Points.vertex_min[i];
            tmp_boxpoints_add.vertex_max[i] = localmap_box_.vertex_min[i];
            cub_needadd.push_back(tmp_boxpoints_add);
        } else if (dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * det_range_){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints_rm.vertex_max[i] = localmap_box_.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints_rm);
            tmp_boxpoints_add.vertex_min[i] = localmap_box_.vertex_max[i];
            tmp_boxpoints_add.vertex_max[i] = New_LocalMap_Points.vertex_max[i];
            cub_needadd.push_back(tmp_boxpoints_add);
        }
    }
    // step5.3 更新localmap地图包围盒
    localmap_box_ = New_LocalMap_Points;
    // step5.4 删除待删除点
    PointVector points_history;
    ikdtree_->acquire_removed_points(points_history);
    // step5.5 根据cub_needrm删除对应localmap
    if(cub_needrm.size() > 0) ikdtree_->Delete_Point_Boxes(cub_needrm);

}

void LaserMapping::Run(){
    // Step 1. sync sensor deque
    if (!SyncPackages()) {
        return;
    }
    
    // Step 2.IMU process, kf prediction, get undistortion lidar points in tail lidar_body frame
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }
    state_point_ = kf_.get_x();
    pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
    LOG(INFO) << "time: "<< std::to_string(measures_.lidar_end_time_);
    LOG(WARNING) << "imu pos: " << state_point_.pos.transpose();

    // Step 3. segment ikd tree fov
    Timer::Evaluate(
        [&, this]() {
            FovSegment();
        },
        "Fov Segment");
    LOG(INFO) << "bbx: " << localmap_box_.vertex_min[0] << " " << localmap_box_.vertex_min[1] << " " << localmap_box_.vertex_min[2]
        << " " << localmap_box_.vertex_max[0] << " " << localmap_box_.vertex_max[1] << " " << localmap_box_.vertex_max[2];
}

} // namespace fast_lio