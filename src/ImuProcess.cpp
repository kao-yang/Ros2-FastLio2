#include "ImuProcess.h"

// namespace humanoid_slam {
// namespace lidar_odom {
// namespace ikd_odom {
namespace fast_lio{

constexpr int MAX_INI_COUNT = 20;

bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); }

ImuProcess::ImuProcess() : m_bFirstFrame(true), m_bImuNeedInit(true) {
    m_nInitIterNum = 1;
    m_Q = process_noise_cov();
    m_covAcc = common::V3D(0.1, 0.1, 0.1);
    m_covGyr = common::V3D(0.1, 0.1, 0.1);
    m_covBiasGyr = common::V3D(0.0001, 0.0001, 0.0001);
    m_covBiasAcc = common::V3D(0.0001, 0.0001, 0.0001);
    m_meanAcc = common::V3D(0, 0, -1.0);
    m_meanGyr = common::V3D(0, 0, 0);
    m_lastAngvel = common::Zero3d;
    m_imuToLidarT = common::Zero3d;
    m_imuToLidarR = common::Eye3d;
    m_pLastImu.reset( new ::humanoid_slam::sensor::ImuData() );
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    m_meanAcc = common::V3D(0, 0, -1.0);
    m_meanGyr = common::V3D(0, 0, 0);
    m_lastAngvel = common::Zero3d;
    m_bImuNeedInit = true;
    m_nInitIterNum = 1;
    m_qPImu.clear();
    vImuPose.clear();
    m_pLastImu.reset( new ::humanoid_slam::sensor::ImuData() );
    m_pUndistortionCloud.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const common::V3D &transl, const common::M3D &rot) {
    m_imuToLidarT = transl;
    m_imuToLidarR = rot;
}

void ImuProcess::SetGyrCov(const common::V3D &scaler) { m_covGyrScale = scaler; }

void ImuProcess::SetAccCov(const common::V3D &scaler) { m_covAccScale = scaler; }

void ImuProcess::SetGyrBiasCov(const common::V3D &b_g) { m_covBiasGyr = b_g; }

void ImuProcess::SetAccBiasCov(const common::V3D &b_a) { m_covBiasAcc = b_a; }

void ImuProcess::IMUInit(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, 
                        input_ikfom> &kfState, int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    common::V3D curAcc, curGyr;

    if ( m_bFirstFrame ) {
        Reset();
        N = 1;
        m_bFirstFrame = false;
        const auto &imuAcc = measure.qPImu.front()->linearAcceleration;
        const auto &imuGyr = measure.qPImu.front()->angularVelocity;
        m_meanAcc << imuAcc.x(), imuAcc.y(), imuAcc.z();
        m_meanGyr << imuGyr.x(), imuGyr.y(), imuGyr.z();
    }

    for (const auto &pImu : measure.qPImu) {
        const auto &imuAcc = pImu->linearAcceleration;
        const auto &imuGyr = pImu->angularVelocity;
        curAcc << imuAcc.x(), imuAcc.y(), imuAcc.z();
        curGyr << imuGyr.x(), imuGyr.y(), imuGyr.z();

        m_meanAcc += (curAcc - m_meanAcc) / N;
        m_meanGyr += (curGyr - m_meanGyr) / N;

        m_covAcc =
            m_covAcc * (N - 1.0) / N + (curAcc - m_meanAcc).cwiseProduct(curAcc - m_meanAcc) * (N - 1.0) / (N * N);
        m_covGyr =
            m_covGyr * (N - 1.0) / N + (curGyr - m_meanGyr).cwiseProduct(curGyr - m_meanGyr) * (N - 1.0) / (N * N);

        N++;
    }
    state_ikfom initState = kfState.get_x();
    initState.grav = S2(-m_meanAcc / m_meanAcc.norm() * common::G_m_s2);

    initState.bg = m_meanGyr;
    initState.offset_T_L_I = m_imuToLidarT;
    initState.offset_R_L_I = m_imuToLidarR;
    kfState.change_x(initState);
    // LOG(FATAL) << "ikf init ex_t:" << initState.offset_T_L_I;

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov initP = kfState.get_P();
    initP.setIdentity();
    initP(6, 6) = initP(7, 7) = initP(8, 8) = 0.00001;
    initP(9, 9) = initP(10, 10) = initP(11, 11) = 0.00001;
    initP(15, 15) = initP(16, 16) = initP(17, 17) = 0.0001;
    initP(18, 18) = initP(19, 19) = initP(20, 20) = 0.001;
    initP(21, 21) = initP(22, 22) = 0.00001;
    kfState.change_P(initP);
    m_pLastImu = measure.qPImu.back();
}

/**
 * @brief iekf forward propagation, use imu to get undistort lidar points in tail lidar_body frame
 * 
 * @param[in] measure imu dates and lidar points in once lidar scan
 * @param[in] kfState pose state of iekf
 * @param[out] pUndistortionCloud undistort points output
 */
void ImuProcess::UndistortPcl(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, input_ikfom> &kfState,
                      PointCloudType &pUndistortionCloud) {
    // Step 1.get head and tail time of imu and lidar
    auto qPImu = measure.qPImu;
    qPImu.push_front(m_pLastImu);    // add the imu_ of the last frame-tail to the of current frame-head
    // const double &imuBeginTime = qPImu.front()->fTime;
    const double &imuEndTime = qPImu.back()->fTime;
    const double &pclBeginTime = measure.pLidar->fBeginTime;
    const double &pclEndTime = measure.pLidar->fEndTime;

    // Step 2.sort point clouds by offset time
    pUndistortionCloud = *(measure.pLidar->pPointCloud);
    sort(pUndistortionCloud.points.begin(), pUndistortionCloud.points.end(), time_list);
    // LOG(WARNING) << "points size: " << pUndistortionCloud.points.size();
    // Step 3.Initialize tail IMU pose of last frame
    state_ikfom imuState = kfState.get_x();
    vImuPose.clear();
    vImuPose.push_back(common::set_pose6d(0.0, m_lastAcc, m_lastAngvel, imuState.vel, imuState.pos,
                                          imuState.rot.toRotationMatrix()));

    
    common::V3D angvelAverage, accAverage, accImu, velImu, posImu;
    common::M3D rotImu;

    double dt = 0;

    input_ikfom in;
    // Step 4.go through all imu vector， forward propagation at each imu_ date
    for (auto itImu = qPImu.begin(); itImu < (qPImu.end() - 1); itImu++) {
        auto &&head = *(itImu);
        auto &&tail = *(itImu + 1);
        // Step 4.1 insure: last lidar end time << tail time
        if (tail->fTime < m_fLastLidarEndTime) {
            continue;
        }

        // Step 4.2 get ave imu date
        angvelAverage << 0.5 * (head->angularVelocity.x() + tail->angularVelocity.x()),
            0.5 * (head->angularVelocity.y() + tail->angularVelocity.y()),
            0.5 * (head->angularVelocity.z() + tail->angularVelocity.z());
        accAverage << 0.5 * (head->linearAcceleration.x() + tail->linearAcceleration.x()),
            0.5 * (head->linearAcceleration.y() + tail->linearAcceleration.y()),
            0.5 * (head->linearAcceleration.z() + tail->linearAcceleration.z());
        
        // Step 4.3 acc normalize to G_m_s2 length
        accAverage = accAverage * common::G_m_s2 / m_meanAcc.norm();  // - state_inout.ba;

        // Step 4.4 get dt
        if (head->fTime < m_fLastLidarEndTime) {
            dt = tail->fTime - m_fLastLidarEndTime;
        } else {
            dt = tail->fTime - head->fTime;
        }

        // Step 4.5 forward propagation this imu date
        in.acc = accAverage;
        in.gyro = angvelAverage;
        m_Q.block<3, 3>(0, 0).diagonal() = m_covGyr;
        m_Q.block<3, 3>(3, 3).diagonal() = m_covAcc;
        m_Q.block<3, 3>(6, 6).diagonal() = m_covBiasGyr;
        m_Q.block<3, 3>(9, 9).diagonal() = m_covBiasAcc;
        kfState.predict(dt, m_Q, in);

        // Step 4.6 save the poses at each IMU measurements( in imu_world frame ) by Step4.5
        imuState = kfState.get_x();
        m_lastAngvel = angvelAverage - imuState.bg;
        m_lastAcc = imuState.rot * (accAverage - imuState.ba);
        // remove g, get really acc
        for (int i = 0; i < 3; i++) {
            m_lastAcc[i] += imuState.grav[i];
        }

        double &&offsTime = tail->fTime - pclBeginTime;
        // simple set off_t、acc、ang、vel、trans、rot
        vImuPose.emplace_back(common::set_pose6d(offsTime, m_lastAcc, m_lastAngvel, imuState.vel, imuState.pos,
                                                 imuState.rot.toRotationMatrix()));
    }

    // Step 5.forward to tail lidar point in cur frame
    double note = pclEndTime > imuEndTime ? 1.0 : -1.0;
    dt = note * (pclEndTime - imuEndTime);
    kfState.predict(dt, m_Q, in);

    imuState = kfState.get_x();
    m_pLastImu = measure.qPImu.back();
    m_fLastLidarEndTime = pclEndTime;

    // Step 6.undistort each lidar point (backward propagation)
    if (pUndistortionCloud.points.empty()) {
        return;
    }
    auto itPcl = pUndistortionCloud.points.end() - 1; // tail point
    // LOG(WARNING) << "orign itPcl: " << itPcl->x << " " << itPcl->y << " " << itPcl->z;
    // backward propagation imu pose
    for (auto itKp = vImuPose.end() - 1; itKp != vImuPose.begin(); itKp--) {
        auto head = itKp - 1;
        auto tail = itKp;
        // rotImu = humanoid_slam::lidar_odom::ikd_odom::common::MatFromArray(head->rot);
        // velImu = humanoid_slam::lidar_odom::ikd_odom::common::VecFromArray(head->vel);
        // posImu = humanoid_slam::lidar_odom::ikd_odom::common::VecFromArray(head->pos);
        // accImu = humanoid_slam::lidar_odom::ikd_odom::common::VecFromArray(tail->acc);
        // angvelAverage = humanoid_slam::lidar_odom::ikd_odom::common::VecFromArray(tail->gyr);
        rotImu = head->rot;
        velImu = head->vel;
        posImu = head->pos;
        accImu = tail->acc;
        angvelAverage = tail->gyr;
        // LOG(INFO) << "imu pose vector offset time: " << head->offset_time;
        // go through points between head and tail imu pose
        for (; (itPcl->curvature / double(1000)) > head->offset_time; itPcl--) {
            dt = itPcl->curvature / double(1000) - head->offset_time;
            // LOG(WARNING) << "dt: " << dt;
            /* ! Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            
            // this point time rot in imu_world
            common::M3D R_i(rotImu * Exp(angvelAverage, dt));
            // this point time translate in lidar_body frame
            common::V3D P_i(itPcl->x, itPcl->y, itPcl->z);
            // delta translate of (end time pose) to (this point time) in imu_world frame
            common::V3D T_ei(posImu + velImu * dt + 0.5 * accImu * dt * dt - imuState.pos);
            // 
            common::V3D p_compensate =
                /*lidar point pose in tail lidar_body frame without rotation distort*/
                imuState.offset_R_L_I.conjugate() *
                /*lidar point pose in tail imu_body frame without rotation distort*/
                (imuState.rot.conjugate() * 
                    /*lidar point pose in imu_world frame with distort*/
                    (R_i * 
                        /*lidar point pose in imu_body frame*/
                        (imuState.offset_R_L_I * P_i + imuState.offset_T_L_I)
                    + T_ei) -
                 imuState.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            itPcl->x = p_compensate(0);
            itPcl->y = p_compensate(1);
            itPcl->z = p_compensate(2);
            // LOG(WARNING) << "itPcl: " << itPcl->x << " " << itPcl->y << " " << itPcl->z;
            if (itPcl == pUndistortionCloud.points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, input_ikfom> &kfState,
                 PointCloudType::Ptr m_pUndistortionCloud) {
    if (measure.qPImu.empty()) {
        return;
    }

    CHECK(measure.pLidar != nullptr);

    if (m_bImuNeedInit) {
        /// The very first lidar frame
        IMUInit(measure, kfState, m_nInitIterNum);

        m_bImuNeedInit = true;

        m_pLastImu = measure.qPImu.back();

        state_ikfom imuState = kfState.get_x();
        if (m_nInitIterNum > MAX_INI_COUNT) {
            m_covAcc *= pow(common::G_m_s2 / m_meanAcc.norm(), 2);
            m_bImuNeedInit = false;

            m_covAcc = m_covAccScale;
            m_covGyr = m_covGyrScale;
            // LOG(WARNING) << "mean Acc: " << m_meanAcc.transpose();
            // LOG(WARNING) << "cov Acc: " << m_covAcc.transpose();
            // LOG(WARNING) << "cov Gyro: " << m_covGyr.transpose();
            LOG(WARNING) << "IMU Initial Done";
        }

        return;
    }
    // UndistortPcl(measure, kfState, *pUndistortionCloud);
    Timer::Evaluate([&, this]() { UndistortPcl(measure, kfState, *m_pUndistortionCloud); }, "Undistort Pcl");
}

}
// }
// }