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
        imu_topic_, 1000, std::bind(&LaserMapping::IMUCallBack, this, std::placeholders::_1));
    sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, 100, std::bind(&LaserMapping::StandardPCLCallBack, this, std::placeholders::_1));
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
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
        // m_bTimeSyncEn = yaml["common"]["time_sync_en"].as<bool>();
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
        m_fCubeLen = yaml["mapping"]["cube_side_length"].as<double>();
        filter_size_surf_min = yaml["mapping"]["filter_size_surf"].as<double>();
        m_fFilterSizeMapMin = yaml["mapping"]["filter_size_map"].as<float>();
        options::NUM_MAX_ITERATIONS = yaml["mapping"]["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["mapping"]["esti_plane_threshold"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        m_fLidarDetectRange = yaml["mapping"]["det_range"].as<float>();
        m_bExtrinsicEstEn = yaml["mapping"]["extrinsic_est_en"].as<bool>();
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

    m_voxelScan.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
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
    if (timestamp < m_fLastImuTime) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    m_fLastImuTime = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
    Run();
}

void LaserMapping::PublishOdometry(){
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = common::ToRosTime(lidar_end_time_);
    odom.pose.pose.position.x = state_point_.pos.x();
    odom.pose.pose.position.y = state_point_.pos.y();
    odom.pose.pose.position.z = state_point_.pos.z();
    odom.pose.pose.orientation.w = state_point_.rot.w();
    odom.pose.pose.orientation.x = state_point_.rot.x();
    odom.pose.pose.orientation.y = state_point_.rot.y();
    odom.pose.pose.orientation.z = state_point_.rot.z();
    pub_odom_->publish(odom);
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = m_pCloudDsInLidarBodyFrame->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            for(size_t i=0; i<cnt_pts; ++i){
                PointType &point_body = m_pCloudDsInLidarBodyFrame->points[i];
                PointType &point_world = m_pCloudDsInMapFrame->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                vector<float> pointSearchSqDis(options::NUM_MATCH_POINTS);
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ikdtree_->Nearest_Search(point_world, options::NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            }//);
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = m_pCloudDsInLidarBodyFrame->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }
    LOG(INFO) << "effect points num: " << effect_feat_num_;
    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            // std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            for( size_t i=0; i<effect_feat_num_; ++i ){
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (m_bExtrinsicEstEn) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            }//);
        },
        "    ObsModel (IEKF Build Jacobian)");
}


bool LaserMapping::SyncPackages(){
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** get new lidar scan ***/
    if (!m_bLidarPushed) {
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

        m_bLidarPushed = true;
    }

    if (m_fLastImuTime < lidar_end_time_) {
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
    m_bLidarPushed = false;
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
    if(!m_bBbxInited){
        for (int i = 0; i < 3; i++){
            localmap_box_.vertex_min[i] = pos_lid(i) - m_fCubeLen / 2.0;
            localmap_box_.vertex_max[i] = pos_lid(i) + m_fCubeLen / 2.0;
        }
        m_bBbxInited = true;
        return;
    }
    // step4. 求取距离和是否需要移动
    float dist_to_map_edge[3][2];   // lidar与立方体六个面的距离
    bool need_move = false;         // 是否需要移动
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_lid(i) - localmap_box_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lid(i) - localmap_box_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange 
           || dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange)
            need_move = true;
    }
    if (!need_move) return;

    // step5. 进行移动localmap操作
    BoxPointType New_LocalMap_Points, tmp_boxpoints_rm, tmp_boxpoints_add;  // 新的局部地图盒子边界点
    New_LocalMap_Points = localmap_box_;
    // step5.1 求取移动距离
    float mov_dist = max((m_fCubeLen - 2.0 * options::MOV_THRESHOLD * m_fLidarDetectRange) * 0.5 * 0.9, 
                          double(m_fLidarDetectRange * (options::MOV_THRESHOLD -1)));
    // step5.2 求取待删除的矩形大小
    for (int i = 0; i < 3; i++){
        tmp_boxpoints_rm = localmap_box_;
        tmp_boxpoints_add = localmap_box_;
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints_rm.vertex_min[i] = localmap_box_.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints_rm);
            tmp_boxpoints_add.vertex_min[i] = New_LocalMap_Points.vertex_min[i];
            tmp_boxpoints_add.vertex_max[i] = localmap_box_.vertex_min[i];
            cub_needadd.push_back(tmp_boxpoints_add);
        } else if (dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
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

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = m_pCloudDsInLidarBodyFrame->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }
    // step1.遍历w系下雷达扫描点
    // std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
    for( size_t i=0; i<cur_pts; ++i ){
        PointBodyToWorld(&(m_pCloudDsInLidarBodyFrame->points[i]), &(m_pCloudDsInMapFrame->points[i]));
        PointType &point_world = m_pCloudDsInMapFrame->points[i];
        // 判断地图上的邻近点不未空，且ekf初始化成功
        if (!nearest_points_[i].empty() && m_bEkfInited) {
            // step1.1 是
            const PointVector &points_near = nearest_points_[i];
            // step1.1.1 求该点所在地图栅格的中心坐标
            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / m_fFilterSizeMapMin).array().floor() + 0.5) * m_fFilterSizeMapMin;
            // step1.1.2 求该点的最近邻点距地图栅格的中心的距离
            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;
            // step1.1.3 根据最近邻点距离判断，该点位置不在对应栅格中，则压入point_no_need_downsample
            if (fabs(dis_2_center.x()) > 0.5 * m_fFilterSizeMapMin &&
                fabs(dis_2_center.y()) > 0.5 * m_fFilterSizeMapMin &&
                fabs(dis_2_center.z()) > 0.5 * m_fFilterSizeMapMin) {
                point_no_need_downsample.emplace_back(point_world);
                // return;
                continue;
                // TODO
            }
            // step1.1.4 求该点与栅格中心的欧式距离
            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            // step1.1.5 如果该点的最近邻点数量>5,遍历邻近点，如果邻近点与栅格中心距离小于该点距离，则不添加(只需要里中心最近的点)
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {// step1.1 否，即第一帧，则直接加入
            points_to_add.emplace_back(point_world);
        }
    }//);

    int add_point_size = ikdtree_->Add_Points(points_to_add,true);
    ikdtree_->Add_Points(point_no_need_downsample,false);
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
    // LOG(INFO) << "bbx: " << localmap_box_.vertex_min[0] << " " << localmap_box_.vertex_min[1] << " " << localmap_box_.vertex_min[2]
        // << " " << localmap_box_.vertex_max[0] << " " << localmap_box_.vertex_max[1] << " " << localmap_box_.vertex_max[2];

    // Step 4. downsample cur body frame lidar scan 
    Timer::Evaluate(
        [&, this]() {
            m_voxelScan.setInputCloud(scan_undistort_); // scan_undistort_
            m_voxelScan.filter(*m_pCloudDsInLidarBodyFrame);
        },
        "Downsample PointCloud");
    int cur_pts = m_pCloudDsInLidarBodyFrame->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << m_pCloudDsInLidarBodyFrame->size();
        return;
    }

    m_pCloudDsInMapFrame->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());
    // Step 5.the first scan,init
    if (m_bFirstScan) {
        if (ikdtree_->Root_Node == nullptr){
            ikdtree_->set_downsample_param(m_fFilterSizeMapMin);
            for(int i = 0; i < cur_pts; i++)
                PointBodyToWorld(&(m_pCloudDsInLidarBodyFrame->points[i]), &(m_pCloudDsInMapFrame->points[i]));
            ikdtree_->Build(m_pCloudDsInMapFrame->points);
        }

        m_fFirstLidarTime = measures_.lidar_bag_time_;
        m_bFirstScan = false;
        return;
    }
    m_bEkfInited = (measures_.lidar_bag_time_ - m_fFirstLidarTime) >= options::INIT_TIME;

    // Step 6.ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
            LOG(WARNING) << "pose after measure: " << state_point_.pos.transpose();
        },
        "IEKF Solve and Update");
    PublishOdometry();

    // Step 7.update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");
}

// tools

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

} // namespace fast_lio