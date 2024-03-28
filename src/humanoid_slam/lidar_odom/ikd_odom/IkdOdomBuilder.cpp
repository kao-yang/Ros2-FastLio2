#include "humanoid_slam/lidar_odom/ikd_odom/IkdOdomBuilder.h"

namespace humanoid_slam{
namespace lidar_odom{
namespace ikd_odom{

IkdOdomBuilder::IkdOdomBuilder( const std::string& sParamsFile ){
    // Step 1. init shared ptr
    m_pIkdTree.reset(new KD_TREE<PointType>());
    m_pImu.reset(new ImuProcess());
    // Step 2. load params
    if ( !LoadParams(sParamsFile) ){
        LOG(FATAL) << "load ikd odom params fail, process shut down";
    }else{
        LOG(INFO) << "load ikd odom parsms file: " << sParamsFile << " success";
    }

    std::vector<double> epsi(23, 0.001);
    m_kf.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &ikfState, esekfom::dyn_share_datastruct<double> &ikfH) { ObsModel(ikfState, ikfH); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    // state_ikfom init_state;
    // Eigen::Vector3d init_rot_rad(0,15,0); // deg
    // init_state.rot = SO3::exp(init_rot_rad/57.53);
    // m_kf.change_x(init_state);
    // TODO load map
    m_bPoseInited = true;

}

IkdOdomBuilder::~IkdOdomBuilder(){
    ::humanoid_slam::lidar_odom::ikd_odom::Timer::PrintAll(); 
}

/**
 * @brief sync liadr and imu
 * 
 * @return true 
 * @return false 
 */
bool IkdOdomBuilder::SyncMeasure(){
    // Step 0. lock
    std::unique_lock<std::mutex> lock(m_inputMtx);
    
    // Step 1. not be empty
    if (m_qLidarDeque.empty() || m_qImuDeque.empty()) {
        return false;
    }

    // Step 2. get new lidar
    if ( !m_bLidarPushed ){
        m_measure.pLidar =  m_qLidarDeque.front();
        m_bLidarPushed = true;
    }

    // Step 3. make sure imu time is small than measure lidar
    if ( m_qImuDeque.back()->fTime < m_measure.pLidar->fEndTime ){
        return false;
    }

    // Step 4. get imu queue
    double fImuTime = m_qImuDeque.front()->fTime;
    m_measure.qPImu.clear();
    while( !m_qImuDeque.empty() && fImuTime <= m_measure.pLidar->fEndTime ){
        fImuTime = m_qImuDeque.front()->fTime;
        if (fImuTime > m_measure.pLidar->fEndTime) { break; }
        m_measure.qPImu.push_back(m_qImuDeque.front());
        m_qImuDeque.pop_front();
    }

    // Step 5. return true
    m_qLidarDeque.pop_front();
    m_bLidarPushed = false;
    return true;
}

/**
 * @brief update ikd tree local map fov after state update
 * 
 */
void IkdOdomBuilder::FovSegment(){
    // Step 1. declare cub vector which need remove and need add
    vector<BoxPointType> vCubNeedRemove;
    vector<BoxPointType> vCubNeedAdd;
    vCubNeedRemove.clear();
    vCubNeedAdd.clear();
    
    // Step 2. get lidar pose in map frame after iekf state update
    vect3 poseLidar = m_lidarPoseInMapFrame;

    // Step 3. init bounding box
    if(!m_bBbxInited){
        for (int i = 0; i < 3; i++){
            m_localMapBbx.vertex_min[i] = poseLidar(i) - m_fCubeLen / 2.0;
            m_localMapBbx.vertex_max[i] = poseLidar(i) + m_fCubeLen / 2.0;
        }
        m_bBbxInited = true;
        return;
    }

    // Step 4. compute fov
    float fDistToMapEdge[3][2];     // distance to local map 6 faces
    bool bNeedMove = false;         // weather need to move fov
    for (int i = 0; i < 3; i++){
        fDistToMapEdge[i][0] = fabs(poseLidar(i) - m_localMapBbx.vertex_min[i]);
        fDistToMapEdge[i][1] = fabs(poseLidar(i) - m_localMapBbx.vertex_max[i]);
        if (fDistToMapEdge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange 
           || fDistToMapEdge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange)
            bNeedMove = true;
    }
    if (!bNeedMove) return;

    // Step 5. move fov
    BoxPointType newLocalMapBbx = m_localMapBbx;
    BoxPointType tmpBbxRemove;
    BoxPointType tmpBbxAdd;
    // Step 5.1 compute move distance
    float fMoveDist = max((m_fCubeLen - 2.0 * options::MOV_THRESHOLD * m_fLidarDetectRange) * 0.5 * 0.9, 
                          double(m_fLidarDetectRange * (options::MOV_THRESHOLD -1)));
    // Step 5.2 compute cube need to remove and need to add( history map) 
    for (int i = 0; i < 3; i++){
        tmpBbxRemove = m_localMapBbx;
        tmpBbxAdd = m_localMapBbx;
        if (fDistToMapEdge[i][0] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
            newLocalMapBbx.vertex_max[i] -= fMoveDist;
            newLocalMapBbx.vertex_min[i] -= fMoveDist;
            tmpBbxRemove.vertex_min[i] = m_localMapBbx.vertex_max[i] - fMoveDist;
            vCubNeedRemove.push_back(tmpBbxRemove);
            tmpBbxAdd.vertex_min[i] = newLocalMapBbx.vertex_min[i];
            tmpBbxAdd.vertex_max[i] = m_localMapBbx.vertex_min[i];
            vCubNeedAdd.push_back(tmpBbxAdd);
        } else if (fDistToMapEdge[i][1] <= options::MOV_THRESHOLD * m_fLidarDetectRange){
            newLocalMapBbx.vertex_max[i] += fMoveDist;
            newLocalMapBbx.vertex_min[i] += fMoveDist;
            tmpBbxRemove.vertex_max[i] = m_localMapBbx.vertex_min[i] + fMoveDist;
            vCubNeedRemove.push_back(tmpBbxRemove);
            tmpBbxAdd.vertex_min[i] = m_localMapBbx.vertex_max[i];
            tmpBbxAdd.vertex_max[i] = newLocalMapBbx.vertex_max[i];
            vCubNeedAdd.push_back(tmpBbxAdd);
        }
    }
    // Step 5.3 update local map bounding box
    m_localMapBbx = newLocalMapBbx;

    // Step 6. remove points
    // Step 6.1 remove local map points ? maybe lazy delete
    PointVector pointsRemoved;
    m_pIkdTree->acquire_removed_points(pointsRemoved);
    // Step 6.2 remove bbx points
    if(vCubNeedRemove.size() > 0) m_pIkdTree->Delete_Point_Boxes(vCubNeedRemove);
    
    // Step 7. add points
    // Step 7.1 init pcl filter by bbx needed add
    if ( !m_bLoadPcdMap ) return;
    pcl::ConditionalRemoval<PointType> pclConditionFilter;
    pcl::ConditionOr<PointType>::Ptr conditionOr(new pcl::ConditionOr<PointType>());
    for( auto &vec : vCubNeedAdd ){
        pcl::ConditionAnd<PointType>::Ptr conditionAdd(new pcl::ConditionAnd<PointType>());
        for( int i=0; i<3; ++i ){
            std::string sAxis;
            if (i==0){ sAxis = "x"; }
            else if(i==1){ sAxis = "y"; }
            else{ sAxis = "z";}

            conditionAdd->addComparison(pcl::FieldComparison<PointType>::ConstPtr
                (new pcl::FieldComparison<PointType>(sAxis, pcl::ComparisonOps::GE, vec.vertex_min[i] )));
            conditionAdd->addComparison(pcl::FieldComparison<PointType>::ConstPtr
                (new pcl::FieldComparison<PointType>(sAxis, pcl::ComparisonOps::LE, vec.vertex_max[i] )));
        }
        conditionOr->addCondition( conditionAdd );
    }
    pclConditionFilter.setCondition(conditionOr);

    // Step 7.2 filter history map
    pclConditionFilter.setInputCloud(m_pPcdCloud);
    CloudPtr pPcdBbxCloud{ new PointCloudType() };
    pclConditionFilter.filter(*pPcdBbxCloud);

    // Step 7.3 add history map points to ikd tree
    if ( pPcdBbxCloud->points.size() > 0 ){
        PointVector vPointAdd;
        vPointAdd.reserve(pPcdBbxCloud->points.size());
        for( auto &p : pPcdBbxCloud->points ){
            vPointAdd.emplace_back(p);
        }
        m_pIkdTree->Add_Points(vPointAdd,true);
    }

}

bool IkdOdomBuilder::RunOdom() {
    // Step 0. pose should be init, TODO change after step 1
    if ( !m_bPoseInited ){
        return false;
    }

    // Step 1. get sync imu deque and lidar scan
    if ( !SyncMeasure() ){
        return false;
    }

    LOG(INFO) << "lidar size: " << m_measure.pLidar->pPointCloud->points.size() 
        << " start time: " << std::to_string(m_measure.pLidar->fBeginTime) << " end time: " << std::to_string(m_measure.pLidar->fEndTime);

    // Step 2. IMU process, kf prediction, get undistortion lidar points in tail lidar_body frame
    m_pImu->Process(m_measure, m_kf, m_pUndistorCloud);
    if (m_pUndistorCloud->empty() || (m_pUndistorCloud == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    // Step 3. move local map fov
    m_ikfState = m_kf.get_x();
    m_lidarPoseInMapFrame = m_ikfState.pos + m_ikfState.rot * m_ikfState.offset_T_L_I;
    // LOG(INFO) << "origin bbx: " << m_localMapBbx.vertex_min[0] << " " << m_localMapBbx.vertex_min[1] << " " << m_localMapBbx.vertex_min[2]
        // << " " << m_localMapBbx.vertex_max[0] << " " << m_localMapBbx.vertex_max[1] << " " << m_localMapBbx.vertex_max[2];
    LOG(INFO) << "imu pose: " << m_ikfState.pos.transpose();
    Timer::Evaluate( [&, this]() { FovSegment(); }, "Fov Segment" );
    // LOG(INFO) << "bbx: " << m_localMapBbx.vertex_min[0] << " " << m_localMapBbx.vertex_min[1] << " " << m_localMapBbx.vertex_min[2]
        // << " " << m_localMapBbx.vertex_max[0] << " " << m_localMapBbx.vertex_max[1] << " " << m_localMapBbx.vertex_max[2];

    // Step 4. downsample cur body frame lidar scan
    Timer::Evaluate(
        [&, this]() {
            m_voxelScan.setInputCloud(m_pUndistorCloud); // m_pUndistorCloud   m_measure.pLidar->pPointCloud
            m_voxelScan.filter(*m_pCloudDsInLidarBodyFrame);
        }, "Downsample PointCloud");
    size_t nCurPointsNum = m_pCloudDsInLidarBodyFrame->size();
    // LOG(INFO) << "ds points num: " << nCurPointsNum;
    if (nCurPointsNum < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << m_pUndistorCloud->size() << ", " << nCurPointsNum;
        return false;
    }

    // Step 5. re init for measure update
    m_pCloudDsInMapFrame->clear();
    m_vNearestPoints.clear();
    m_vResiduals.clear();
    m_vPointSelectedSurf.clear();
    m_planeCoef.clear();

    m_pCloudDsInMapFrame->resize(nCurPointsNum);
    m_vNearestPoints.resize(nCurPointsNum);
    m_vResiduals.resize(nCurPointsNum, 0);
    m_vPointSelectedSurf.resize(nCurPointsNum, true);
    m_planeCoef.resize(nCurPointsNum, common::V4F::Zero());

    // Step 6. the first scan, init
    if (m_bFirstScan) {
        if (m_pIkdTree->Root_Node == nullptr){
            m_pIkdTree->set_downsample_param(m_fFilterSizeMapMin);
            for(size_t i = 0; i < nCurPointsNum; i++)
                PointBodyToWorld(&(m_pCloudDsInLidarBodyFrame->points[i]), &(m_pCloudDsInMapFrame->points[i]));
            m_pIkdTree->Build(m_pCloudDsInMapFrame->points);
        }
        m_fFirstLidarTime = m_measure.pLidar->fEndTime;
        m_bFirstScan = false;
        return false;
    }
    // m_bEkfInited = (m_measure.pLidar->fEndTime - m_fFirstLidarTime) >= options::INIT_TIME;
    m_bEkfInited = (m_measure.pLidar->fEndTime - m_fFirstLidarTime) >= 1;

    // Step 7. ICP and iterated Kalman Filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double fSolveHmatrixTime = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            m_kf.update_iterated_dyn_share_modified(options::LASER_POINT_COV, fSolveHmatrixTime);
            // save the state
            m_ikfState = m_kf.get_x();
            m_lidarPoseInMapFrame = m_ikfState.pos + m_ikfState.rot * m_ikfState.offset_T_L_I;
            LOG(WARNING) << "pose after measure: " << m_ikfState.pos.transpose();
        },
        "IEKF Solve and Update");

    // Step 8. update local map
    if ( !m_bLoadPcdMap || m_bUpdatePcdMap ){
        if (m_bEkfInited){
            Timer::Evaluate([&, this]() { MapIncremental(); }, "Incremental Mapping");
        }
            
    }

    // Step 9. update lidar odom data output
    std::unique_lock<std::mutex> lock(m_outputMtx);
    m_undistortCloud = ::humanoid_slam::sensor::TimedPointCloudData
                    {m_measure.pLidar->fBeginTime, m_measure.pLidar->fEndTime, m_pUndistorCloud};

    // Timer::Evaluate(
    //     [&, this]() {
    //         PointVector ().swap(m_pIkdTree->PCL_Storage);
    //         m_pIkdTree->flatten(m_pIkdTree->Root_Node, m_pIkdTree->PCL_Storage, NOT_RECORD);
    //         m_pIkdCloud->clear();
    //         m_pIkdCloud->points = m_pIkdTree->PCL_Storage;
    //         m_localMap = ::humanoid_slam::sensor::TimedPointCloudData
    //                         {m_measure.pLidar->fBeginTime, m_measure.pLidar->fEndTime, m_pIkdCloud};
    //     }, "local map from ikd tree");

    m_imuPoseInMapFrame.fTime = m_measure.pLidar->fEndTime;
    m_imuPoseInMapFrame.sFrameId = "map";
    m_imuPoseInMapFrame.sChildFrameId = "imu";
    m_imuPoseInMapFrame.bodyLinearVelocity = m_ikfState.rot * m_ikfState.vel;
    m_imuPoseInMapFrame.bodyAngularVelocity = m_measure.qPImu.back()->angularVelocity;
    m_imuPoseInMapFrame.pose = Sophus::SE3d{
        Eigen::Quaterniond{m_ikfState.rot.w(), m_ikfState.rot.x(), m_ikfState.rot.y(), m_ikfState.rot.z()}, 
                                            m_ikfState.pos};
    lock.unlock();

    return true;
}
/**
 * @brief make sure which points directly to add, which need down sample, then add them to ikd tree
 * 1. no near points, direct add point
 * 2. near points is not in this point voxel, add this points without down sampling
 * 3. no enough near points, or this point distance is shorter to center, add point
 * 4. there are enough points, and near points' distance to center is short to this point, no need add
 */
void IkdOdomBuilder::MapIncremental() {
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
            // get voxel center location of this point
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

/**
 * @brief compute point-to-plane residual, get iekf H matrix
 * 
 * @param[in] ikfState kf state
 * @param[out] ikfH H matrix
 */
void IkdOdomBuilder::ObsModel(state_ikfom &ikfState, esekfom::dyn_share_datastruct<double> &ikfH) {
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
            std::for_each(std::execution::seq, index.begin(), index.end(), [&](const size_t &i) {
            // for( size_t i=0; i<nPointsNum; ++i ){
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
            });
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

void IkdOdomBuilder::InputData( const ::humanoid_slam::sensor::ImuData& imuData) {
    std::unique_lock<std::mutex> lock(m_inputMtx);
    CHECK_GT(imuData.fTime, m_fLastImuTime) << "imu time reverse flow";
    m_fLastImuTime = imuData.fTime;
    m_qImuDeque.emplace_back( std::make_shared<::humanoid_slam::sensor::ImuData>(imuData) );
}


void IkdOdomBuilder::InputData( const ::humanoid_slam::sensor::TimedPointCloudData& cloudData) {
    std::unique_lock<std::mutex> lock(m_inputMtx);
    CHECK_GT(cloudData.fEndTime, m_fLastLidarTime) << "lidar time reverse flow";
    m_fLastLidarTime = cloudData.fEndTime;
    m_qLidarDeque.emplace_back( std::make_shared<::humanoid_slam::sensor::TimedPointCloudData>(cloudData) );
}

::humanoid_slam::sensor::TimedPointCloudData IkdOdomBuilder::GetUndistorCloud(){
    std::unique_lock<std::mutex> lock(m_outputMtx);
    return m_undistortCloud;
}

::humanoid_slam::sensor::TimedPointCloudData IkdOdomBuilder::GetLocalMap(){
    std::unique_lock<std::mutex> lock(m_outputMtx);
    return m_localMap;
}

::humanoid_slam::sensor::PoseData IkdOdomBuilder::GetLocalPose(){
    std::unique_lock<std::mutex> lock(m_outputMtx);
    return m_imuPoseInMapFrame;
}

bool IkdOdomBuilder::LoadParams(const std::string &sParamsFile){
    // Step 1. check file exist
    if ( !std::filesystem::exists(sParamsFile) ){
        LOG(ERROR) << "ikd odom param file: " << sParamsFile << " not exits";
        return false;
    }

    // Step 2.
    double fGyrCov, fAccCov, fBiasGyrCov, fBiasAccCov;
    double fFilterSizeSurfMin;
    std::vector<double> vExtrinT{3, 0.0};
    std::vector<double> vExtrinR{9, 0.0};
    common::V3D imuToLidarT;
    common::M3D imuToLidarR;
    auto yaml = YAML::LoadFile(sParamsFile);
    try{
        m_fCubeLen = yaml["odom"]["cube_side_length"].as<double>();
        fFilterSizeSurfMin = yaml["odom"]["filter_size_surf"].as<double>();
        m_fFilterSizeMapMin = yaml["odom"]["filter_size_map"].as<double>();
        options::NUM_MAX_ITERATIONS = yaml["odom"]["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["odom"]["esti_plane_threshold"].as<double>();
        fAccCov = yaml["odom"]["acc_cov"].as<double>();
        fGyrCov = yaml["odom"]["gyr_cov"].as<double>();
        fBiasAccCov = yaml["odom"]["b_acc_cov"].as<double>();
        fBiasGyrCov = yaml["odom"]["b_gyr_cov"].as<double>();
        m_fLidarDetectRange = yaml["odom"]["lidar_range"].as<float>();
        m_bExtrinsicEstEn = yaml["odom"]["extrinsic_est_en"].as<bool>();
        vExtrinT = yaml["odom"]["extrinsic_T"].as<std::vector<double>>();
        vExtrinR = yaml["odom"]["extrinsic_R"].as<std::vector<double>>();
        m_bTimeSyncEn = yaml["odom"]["time_sync_en"].as<bool>();
        m_bLoadPcdMap = yaml["pcd_map"]["pcd_save_en"].as<bool>();
        m_bSavePcdMap = yaml["pcd_map"]["load_pcd_map"].as<bool>();
        m_bUpdatePcdMap = yaml["pcd_map"]["update_pcd_map"].as<bool>();
    }catch(...){
        LOG(ERROR) << "load ikd odom yaml file: " << sParamsFile << " fail";
        return false;
    }
    m_voxelScan.setLeafSize(fFilterSizeSurfMin, fFilterSizeSurfMin, fFilterSizeSurfMin);
    m_voxelMap.setLeafSize(m_fFilterSizeMapMin, m_fFilterSizeMapMin, m_fFilterSizeMapMin);

    imuToLidarT = common::VecFromArray<double>(vExtrinT);
    imuToLidarR = common::MatFromArray<double>(vExtrinR);
    m_pImu->SetExtrinsic(imuToLidarT, imuToLidarR);
    m_pImu->SetGyrCov(common::V3D(fGyrCov, fGyrCov, fGyrCov));
    m_pImu->SetAccCov(common::V3D(fAccCov, fAccCov, fAccCov));
    m_pImu->SetGyrBiasCov(common::V3D(fBiasGyrCov, fBiasGyrCov, fBiasGyrCov));
    m_pImu->SetAccBiasCov(common::V3D(fBiasAccCov, fBiasAccCov, fBiasAccCov));
    
    LOG(INFO) << "extrin T: " << imuToLidarT.transpose();
    LOG(INFO) << "extrin R: " << imuToLidarR.transpose();
    return true;
}

// tools

void IkdOdomBuilder::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D pointBody(pi->x, pi->y, pi->z);
    common::V3D pointGlobal(m_ikfState.rot * (m_ikfState.offset_R_L_I * pointBody + m_ikfState.offset_T_L_I) +
                         m_ikfState.pos);

    po->x = pointGlobal(0);
    po->y = pointGlobal(1);
    po->z = pointGlobal(2);
    po->intensity = pi->intensity;
    LOG_FIRST_N(WARNING,5) << std::fixed << std::setprecision(6) << "offset_T: " << m_ikfState.offset_T_L_I.transpose() << std::endl
                            << "offser_R: " << std::endl << m_ikfState.offset_R_L_I << std::endl
                            << "point in: " << pi->x << " " << pi->y << " " << pi->z << " " << pi->curvature << std::endl
                            << "point out: " << po->x << " " << po->y << " " << po->z << " " << po->curvature << std::endl;

}

void IkdOdomBuilder::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D pBody(pi.x(), pi.y(), pi.z());
    common::V3D pGlobal(m_ikfState.rot * (m_ikfState.offset_R_L_I * pBody + m_ikfState.offset_T_L_I) +
                         m_ikfState.pos);

    po->x = pGlobal(0);
    po->y = pGlobal(1);
    po->z = pGlobal(2);
    po->intensity = std::abs(po->z);
}

} // namespace ikd_odom
} // namespace lidar_odom
} // namespace humanoid_slam