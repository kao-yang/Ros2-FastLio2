#include "laser_mapping.h"

namespace fast_lio {

LaserMapping::LaserMapping(const std::string& sParamsDir)
        : Node("ros2_fast_lio2")
{
    // Step 0. init ros2 node handle
    node_handler_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    // Step 1.init ptr
    preprocess_.reset( new PointCloudPreprocess() );
    m_pImu.reset( new ImuProcess() );
    m_pIkdTree.reset( new KD_TREE<PointType>() );

    // Step 2.load params
    if ( !LoadParamsFromYAML( sParamsDir+ "ikdodom.yaml") ){
        LOG(FATAL) << "load param fail,process exit";
    }else{
        LOG(INFO) << "load param success";
    }

    // Step 3. init iekf
    std::vector<double> epsi(23, 0.001);
    m_kf.init_dyn_share(
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

    m_pImu->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    m_pImu->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    m_pImu->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    m_pImu->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    m_pImu->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    LOG(WARNING) << "extrin T: " << lidar_T_wrt_IMU.transpose();
    LOG(WARNING) << "extrin R: " << lidar_R_wrt_IMU.transpose();

    return true;
}



void LaserMapping::StandardPCLCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    mtx_buffer_.lock();
    // Timer::Evaluate(
    //     [&, this]() {
    //         PointCloudType::Ptr ptr(new PointCloudType());

    //         sensor_msgs::msg::PointCloud2::SharedPtr changeMsg (new sensor_msgs::msg::PointCloud2(*msg));
    //         // timestamp correct for rs16
    //         if (preprocess_->GetLidarType()==LidarType::RS16){
    //             double timeOffSet = 0;
    //             unsigned char tmp[8];
    //             for(int i=0;i<8;i++){
    //                 tmp[i] = changeMsg->data[18+i];
    //             }
    //             timeOffSet = *(double*)tmp;
    //             changeMsg->header.stamp = fast_lio::common::ToRosTime(timeOffSet);
    //         }
            

    //         preprocess_->Process(changeMsg, ptr);
    //         lidar_buffer_.push_back(ptr);
    //         time_buffer_.push_back( fast_lio::common::FromRosTime(changeMsg->header.stamp) );
    //         // LOG(INFO) << "time buffer input, time: " << std::to_string( fast_lio::common::FromRosTime(changeMsg->header.stamp) );
    //         // LOG(INFO) << "lidar buffer input, size: " << ptr->points.size();

    //         last_timestamp_lidar_ = fast_lio::common::FromRosTime(changeMsg->header.stamp);
    //     },
    //     "Preprocess (Standard)");
    auto cloudData = RsHoliesToTimedPointCloudData(msg);
    lidar_buffer_.push_back(cloudData.pPointCloud);
    time_buffer_.push_back(cloudData.fBeginTime);
    last_timestamp_lidar_ = cloudData.fBeginTime;

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
    odom.pose.pose.position.x = m_ikfState.pos.x();
    odom.pose.pose.position.y = m_ikfState.pos.y();
    odom.pose.pose.position.z = m_ikfState.pos.z();
    odom.pose.pose.orientation.w = m_ikfState.rot.w();
    odom.pose.pose.orientation.x = m_ikfState.rot.x();
    odom.pose.pose.orientation.y = m_ikfState.rot.y();
    odom.pose.pose.orientation.z = m_ikfState.rot.z();
    pub_odom_->publish(odom);
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &ikfState, esekfom::dyn_share_datastruct<double> &ikfH) {
    // Step 0. get points size
    size_t nPointsNum = m_pCloudDsInLidarBodyFrame->size();
    std::vector<size_t> index(nPointsNum);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }


    // Step 1. closest surface search and residual computation
    Timer::Evaluate(
        [&, this]() {
            // get R t from lidar body frame to map frame
            auto rotLidarBodyToMap = (ikfState.rot * ikfState.offset_R_L_I).cast<float>();
            auto transLidarBodyToMap = (ikfState.rot * ikfState.offset_T_L_I + ikfState.pos).cast<float>();

            // closest surface search and residual computation
            // std::for_each(std::execution::seq, index.begin(), index.end(), [&](const size_t &i) {
            for( size_t i=0; i<nPointsNum; ++i ){
                PointType &pointLidarBodyFrame = m_pCloudDsInLidarBodyFrame->points[i];
                PointType &pointMapFrame = m_pCloudDsInMapFrame->points[i];

                // transform to map frame
                common::V3F pLidarBody = pointLidarBodyFrame.getVector3fMap();
                pointMapFrame.getVector3fMap() = rotLidarBodyToMap * pLidarBody + transLidarBodyToMap;
                pointMapFrame.intensity = pointLidarBodyFrame.intensity;

                auto &pointsNear = m_vNearestPoints[i];
                vector<float> pointSearchSqDis(options::NUM_MATCH_POINTS);  // near points to be found
                if (ikfH.converge) {
                    // Find the closest surfaces in the map, filter outlier points // TODO
                    m_pIkdTree->Nearest_Search(pointMapFrame, options::NUM_MATCH_POINTS, pointsNear, pointSearchSqDis);
                    m_vPointSelectedSurf[i] = pointsNear.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (m_vPointSelectedSurf[i]) {
                        m_vPointSelectedSurf[i] =
                            common::esti_plane(m_planeCoef[i], pointsNear, options::ESTI_PLANE_THRESHOLD);
                    }
                }
                // compute residuals: point to plane distance
                if (m_vPointSelectedSurf[i]) {
                    auto temp = pointMapFrame.getVector4fMap();
                    temp[3] = 1.0;
                    float fDistancePointToPlane = m_planeCoef[i].dot(temp);  // the distance
                    // more closer points, more shorter distance, 81 is empirical parameter
                    bool bvalidCorr = pLidarBody.norm() > 81 * fDistancePointToPlane * fDistancePointToPlane;
                    if (bvalidCorr) {
                        m_vPointSelectedSurf[i] = true;
                        m_vResiduals[i] = fDistancePointToPlane;
                    }
                }
            }//);
        },
        "    ObsModel (Lidar Match)");

    // Step 2. update measures(points to planes) before compute H matrix
    m_nEffectFeatureNum = 0;

    m_effectFeaturePoints.resize(nPointsNum);
    m_effectFeatureNormals.resize(nPointsNum);
    for (size_t i = 0; i < nPointsNum; i++) {
        if (m_vPointSelectedSurf[i]) {
            m_effectFeatureNormals[m_nEffectFeatureNum] = m_planeCoef[i];
            m_effectFeaturePoints[m_nEffectFeatureNum] = m_pCloudDsInLidarBodyFrame->points[i].getVector4fMap();
            m_effectFeaturePoints[m_nEffectFeatureNum][3] = m_vResiduals[i];

            m_nEffectFeatureNum++;
        }
    }
    m_effectFeaturePoints.resize(m_nEffectFeatureNum);
    m_effectFeatureNormals.resize(m_nEffectFeatureNum);

    if (m_nEffectFeatureNum < 1) {
        ikfH.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }
    // LOG(INFO) << "effect points num: " << m_nEffectFeatureNum;
    // Step 3. Computation of Measurement Jacobian matrix H and measurements vector
    Timer::Evaluate(
        [&, this]() {
            ikfH.h_x = Eigen::MatrixXd::Zero(m_nEffectFeatureNum, 12);  // ? m_nEffectFeatureNum*12
            ikfH.h.resize(m_nEffectFeatureNum);

            const common::M3F offRotMatrix = ikfState.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F offTrans = ikfState.offset_T_L_I.cast<float>();
            const common::M3F rotMatrix = ikfState.rot.toRotationMatrix().transpose().cast<float>();

            for(size_t i=0; i<m_nEffectFeatureNum; ++i){
                common::V3F pointThisBe = m_effectFeaturePoints[i].head<3>();
                common::M3F pointBeCrossmat = SKEW_SYM_MATRIX(pointThisBe);
                common::V3F pointThis = offRotMatrix * pointThisBe + offTrans;
                common::M3F pointCrossmat = SKEW_SYM_MATRIX(pointThis);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F normVec = m_effectFeatureNormals[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(rotMatrix * normVec);
                common::V3F A(pointCrossmat * C);

                if (m_bExtrinsicEstEn) {
                    common::V3F B(pointBeCrossmat * offRotMatrix.transpose() * C);
                    ikfH.h_x.block<1, 12>(i, 0) << normVec[0], normVec[1], normVec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ikfH.h_x.block<1, 12>(i, 0) << normVec[0], normVec[1], normVec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                // Measurement: distance to the closest surface
                ikfH.h(i) = -m_effectFeaturePoints[i][3];
            }
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
    vect3 pos_lid = m_lidarPoseInMapFrame;
    // step3.初始化box
    if(!m_bBbxInited){
        for (int i = 0; i < 3; i++){
            m_localMapBbx.vertex_min[i] = pos_lid(i) - m_fCubeLen / 2.0;
            m_localMapBbx.vertex_max[i] = pos_lid(i) + m_fCubeLen / 2.0;
        }
        m_bBbxInited = true;
        return;
    }
    // step4. 求取距离和是否需要移动
    float dist_to_map_edge[3][2];   // lidar与立方体六个面的距离
    bool need_move = false;         // 是否需要移动
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_lid(i) - m_localMapBbx.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lid(i) - m_localMapBbx.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange 
           || dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange)
            need_move = true;
    }
    if (!need_move) return;

    // step5. 进行移动localmap操作
    BoxPointType New_LocalMap_Points, tmp_boxpoints_rm, tmp_boxpoints_add;  // 新的局部地图盒子边界点
    New_LocalMap_Points = m_localMapBbx;
    // step5.1 求取移动距离
    float mov_dist = max((m_fCubeLen - 2.0 * options::MOV_THRESHOLD * m_fLidarDetectRange) * 0.5 * 0.9, 
                          double(m_fLidarDetectRange * (options::MOV_THRESHOLD -1)));
    // step5.2 求取待删除的矩形大小
    for (int i = 0; i < 3; i++){
        tmp_boxpoints_rm = m_localMapBbx;
        tmp_boxpoints_add = m_localMapBbx;
        if (dist_to_map_edge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints_rm.vertex_min[i] = m_localMapBbx.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints_rm);
            tmp_boxpoints_add.vertex_min[i] = New_LocalMap_Points.vertex_min[i];
            tmp_boxpoints_add.vertex_max[i] = m_localMapBbx.vertex_min[i];
            cub_needadd.push_back(tmp_boxpoints_add);
        } else if (dist_to_map_edge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints_rm.vertex_max[i] = m_localMapBbx.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints_rm);
            tmp_boxpoints_add.vertex_min[i] = m_localMapBbx.vertex_max[i];
            tmp_boxpoints_add.vertex_max[i] = New_LocalMap_Points.vertex_max[i];
            cub_needadd.push_back(tmp_boxpoints_add);
        }
    }
    // step5.3 更新localmap地图包围盒
    m_localMapBbx = New_LocalMap_Points;
    // step5.4 删除待删除点
    PointVector points_history;
    m_pIkdTree->acquire_removed_points(points_history);
    // step5.5 根据cub_needrm删除对应localmap
    if(cub_needrm.size() > 0) m_pIkdTree->Delete_Point_Boxes(cub_needrm);

}

void LaserMapping::MapIncremental() {
    // init points vector added to ikd tree
    PointVector pointsToAdd;            // points directly to add
    PointVector pointNoNeedDownsample;  // points to add, but no need downsample
    size_t nPointsNum = m_pCloudDsInLidarBodyFrame->size();
    pointsToAdd.reserve(nPointsNum);
    pointNoNeedDownsample.reserve(nPointsNum);

    // multi parallel go through all cur points
    // #pragma omp parallel for num_threads(MP_PROC_NUM)
    for(size_t i=0; i<nPointsNum; ++i){
        // Step 1. transform points to map frame
        PointBodyToWorld(&(m_pCloudDsInLidarBodyFrame->points[i]), &(m_pCloudDsInMapFrame->points[i]));
        PointType &pointInMapFrame = m_pCloudDsInMapFrame->points[i];
        // 
        if ( !m_vNearestPoints[i].empty() && m_bEkfInited ){
            // Step 3. get near point, compute distance of voxel center to near point 
            const PointVector &pointsNear = m_vNearestPoints[i]; // near points with this point
            // get voxel center location of this 
            Eigen::Vector3f center =
                ((pointInMapFrame.getVector3fMap() / m_fFilterSizeMapMin).array().floor() + 0.5) * m_fFilterSizeMapMin;
            // distance between near point to voxel center
            Eigen::Vector3f disToCenter = pointsNear[0].getVector3fMap() - center;
            // Step 4. near points far from center
            // means this voxel is empty, so directly add this points to voxel
            // no need down sampling
            if (fabs(disToCenter.x()) > 0.5 * m_fFilterSizeMapMin &&
                fabs(disToCenter.y()) > 0.5 * m_fFilterSizeMapMin &&
                fabs(disToCenter.z()) > 0.5 * m_fFilterSizeMapMin) {
                pointNoNeedDownsample.emplace_back(pointInMapFrame);
                // return;
                continue;
            }
            // Step 5.if this point is far, and the near points num are enough, not add 
            // else, add this point, and down sampling
            bool bNeedAdd = true;
            float dist = common::calc_dist(pointInMapFrame.getVector3fMap(), center);
            if (pointsNear.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(pointsNear[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        bNeedAdd = false;
                        break;
                    }
                }
            }
            if (bNeedAdd) {
                pointsToAdd.emplace_back(pointInMapFrame);
            }
        } else{
            // Step 2. if no near points, directly add points
            pointsToAdd.emplace_back(pointInMapFrame);
        }
    }

    m_pIkdTree->Add_Points(pointsToAdd,true);
    m_pIkdTree->Add_Points(pointNoNeedDownsample,false);

}

void LaserMapping::Run(){
    // Step 1. sync sensor deque
    if (!SyncPackages()) {
        return;
    }
    
    // Step 2.IMU process, kf prediction, get undistortion lidar points in tail lidar_body frame
    m_pImu->Process(measures_, m_kf, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }
    m_ikfState = m_kf.get_x();
    m_lidarPoseInMapFrame = m_ikfState.pos + m_ikfState.rot * m_ikfState.offset_T_L_I;
    LOG(INFO) << "time: "<< std::to_string(measures_.lidar_end_time_);
    LOG(WARNING) << "imu pos: " << m_ikfState.pos.transpose();

    // Step 3. segment ikd tree fov
    Timer::Evaluate(
        [&, this]() {
            FovSegment();
        },
        "Fov Segment");
    // LOG(INFO) << "bbx: " << m_localMapBbx.vertex_min[0] << " " << m_localMapBbx.vertex_min[1] << " " << m_localMapBbx.vertex_min[2]
        // << " " << m_localMapBbx.vertex_max[0] << " " << m_localMapBbx.vertex_max[1] << " " << m_localMapBbx.vertex_max[2];

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
    m_vNearestPoints.resize(cur_pts);
    m_vResiduals.resize(cur_pts, 0);
    m_vPointSelectedSurf.resize(cur_pts, true);
    m_planeCoef.resize(cur_pts, common::V4F::Zero());
    // Step 5.the first scan,init
    if (m_bFirstScan) {
        if (m_pIkdTree->Root_Node == nullptr){
            m_pIkdTree->set_downsample_param(m_fFilterSizeMapMin);
            for(int i = 0; i < cur_pts; i++)
                PointBodyToWorld(&(m_pCloudDsInLidarBodyFrame->points[i]), &(m_pCloudDsInMapFrame->points[i]));
            m_pIkdTree->Build(m_pCloudDsInMapFrame->points);
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
            m_kf.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            m_ikfState = m_kf.get_x();
            euler_cur_ = SO3ToEuler(m_ikfState.rot);
            m_lidarPoseInMapFrame = m_ikfState.pos + m_ikfState.rot * m_ikfState.offset_T_L_I;
            LOG(WARNING) << "pose after measure: " << m_ikfState.pos.transpose();
        },
        "IEKF Solve and Update");
    PublishOdometry();

    // Step 7.update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");
}

// tools

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(m_ikfState.rot * (m_ikfState.offset_R_L_I * p_body + m_ikfState.offset_T_L_I) +
                         m_ikfState.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(m_ikfState.rot * (m_ikfState.offset_R_L_I * p_body + m_ikfState.offset_T_L_I) +
                         m_ikfState.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}


::humanoid_slam::sensor::TimedPointCloudData
        LaserMapping::RsHoliesToTimedPointCloudData
        ( const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pMsg ){
    // Step 1. get time
    double fEndTime = ::fast_lio::common::FromRosTime( pMsg->header.stamp );

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


} // namespace fast_lio