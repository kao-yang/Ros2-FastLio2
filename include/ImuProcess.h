#pragma once

#include <glog/logging.h>
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"
#include "utils.h"
#include "sensor/LidarOdomMeasureData.h"

// namespace humanoid_slam {
// namespace lidar_odom {
namespace fast_lio {

/// IMU Process and undistortion
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const common::V3D &transl, const common::M3D &rot);
    void SetGyrCov(const common::V3D &scaler);
    void SetAccCov(const common::V3D &scaler);
    void SetGyrBiasCov(const common::V3D &b_g);
    void SetAccBiasCov(const common::V3D &b_a);
    void Process(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, input_ikfom> &kfState,
                 PointCloudType::Ptr pUndistortionCloud);

    Eigen::Matrix<double, 12, 12> m_Q;
    common::V3D m_covAcc;
    common::V3D m_covGyr;
    common::V3D m_covAccScale;
    common::V3D m_covGyrScale;
    common::V3D m_covBiasGyr;
    common::V3D m_covBiasAcc;

   private:
    void IMUInit(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, input_ikfom> &kfState, int &N);
    void UndistortPcl(const ::humanoid_slam::sensor::LidarOdomMeasureData &measure, esekfom::esekf<state_ikfom, 12, input_ikfom> &kfState,
                      PointCloudType &pUndistortionCloud);

    PointCloudType::Ptr m_pUndistortionCloud;
    std::shared_ptr<::humanoid_slam::sensor::ImuData> m_pLastImu;
    std::deque<std::shared_ptr<::humanoid_slam::sensor::ImuData>> m_qPImu;
    std::vector<common::Pose6D> vImuPose;
    std::vector<common::M3D> m_vRotPcl;
    common::M3D m_imuToLidarR;
    common::V3D m_imuToLidarT;
    common::V3D m_meanAcc;
    common::V3D m_meanGyr;
    common::V3D m_lastAngvel;
    common::V3D m_lastAcc;

    double m_fLastLidarEndTime = 0;
    int m_nInitIterNum = 1;
    bool m_bFirstFrame = true;
    bool m_bImuNeedInit = true;
};



}  // namespace ikd_odom
// }  // namespace lidar_odom
// }  // namespace humanoid_slam


