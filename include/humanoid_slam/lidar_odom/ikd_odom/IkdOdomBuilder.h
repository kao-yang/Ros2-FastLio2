#pragma once

#include <filesystem>
#include <mutex>
#include <deque>
#include <vector>
#include <algorithm>
#include <execution>
#include <iomanip>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include "humanoid_slam/lidar_odom/LidarOdomInterface.h"
#include "humanoid_slam/sensor/LidarOdomMeasureData.h"
#include "humanoid_slam/sensor/PoseData.h"

#include "Options.h"
#include "IkdTree.h"
#include "ImuProcess.h"

namespace humanoid_slam{
namespace lidar_odom{
namespace ikd_odom{
class IkdOdomBuilder : public LidarOdomInterface{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit IkdOdomBuilder( const std::string& sParamsFile );
    ~IkdOdomBuilder()override;

    IkdOdomBuilder(const IkdOdomBuilder&) = delete;
    IkdOdomBuilder& operator=(const IkdOdomBuilder&) = delete;

    bool RunOdom() override;
    void InputData( const ::humanoid_slam::sensor::ImuData& imuData) override;
    void InputData( const ::humanoid_slam::sensor::TimedPointCloudData& cloudData) override;

    ::humanoid_slam::sensor::TimedPointCloudData GetUndistorCloud() override;
    ::humanoid_slam::sensor::TimedPointCloudData GetLocalMap() override;
    ::humanoid_slam::sensor::PoseData GetLocalPose() override;

    bool LoadParams(const std::string &sParamsFile);

    bool SyncMeasure();

    // interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &ikfState, esekfom::dyn_share_datastruct<double> &ikfH);
    void FovSegment();
    void MapIncremental();

private:
    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);

private:
    // mutex
    std::mutex m_inputMtx;
    std::mutex m_outputMtx;
    // params
    double m_fCubeLen = 0;
    double m_fFilterSizeMapMin = 0;
    float m_fLidarDetectRange = 150.f;
    bool m_bExtrinsicEstEn = true;
    bool m_bTimeSyncEn = false;
    bool m_bSavePcd = false;
    bool m_bLoadPcdMap = false;
    bool m_bSavePcdMap = false;
    bool m_bUpdatePcdMap = false;

    // pcl voxel filter
    pcl::VoxelGrid<PointType> m_voxelScan;
    pcl::VoxelGrid<PointType> m_voxelMap;

    // input data queue
    std::deque< std::shared_ptr<::humanoid_slam::sensor::TimedPointCloudData> > m_qLidarDeque;
    std::deque< std::shared_ptr<::humanoid_slam::sensor::ImuData> > m_qImuDeque;
    ::humanoid_slam::sensor::LidarOdomMeasureData m_measure;
    bool m_bLidarPushed = false;
    double m_fLastImuTime = -1;
    double m_fLastLidarTime = -1;

    // middle data
    CloudPtr m_pUndistorCloud{ new PointCloudType() };
    CloudPtr m_pCloudDsInLidarBodyFrame{ new PointCloudType() };
    CloudPtr m_pCloudDsInMapFrame{ new PointCloudType() };
    CloudPtr m_pPcdCloud{ new PointCloudType() };           // cloud from pcd file
    CloudPtr m_pIkdCloud{ new PointCloudType() };

    bool m_bFirstScan = true;                           // weather first scan for lidar odom, to init ikd tree
    bool m_bPoseInited = false;                         // weather get init pose(when load history map)
    bool m_bBbxInited = false;                         // weather ikd tree local map bounding box
    bool m_bEkfInited = false;
    double m_fFirstLidarTime = 0.0;
    BoxPointType m_localMapBbx;                         // ikd tree local map bounding box
    std::vector<PointVector> m_vNearestPoints;          // nearest points of current scan, by ikd tree search
    std::vector<bool> m_vPointSelectedSurf;             // selected points
    common::VV4F m_planeCoef;                           // plane coeffs
    std::vector<float> m_vResiduals;                    // point-to-plane residuals
    size_t m_nEffectFeatureNum = 0;                        // effect lidar point num jointed ekf
    common::VV4F m_effectFeaturePoints;                 // inlier points location and residuals
    common::VV4F m_effectFeatureNormals;                // inlier plane norms
    state_ikfom m_ikfState;                             // iekf (imu) state
    vect3 m_lidarPoseInMapFrame;                        // iekf lidar pose

    // output data
    ::humanoid_slam::sensor::TimedPointCloudData m_undistortCloud;      // scan after undistortion
    ::humanoid_slam::sensor::TimedPointCloudData m_localMap;            // local ikd map
    ::humanoid_slam::sensor::PoseData m_imuPoseInMapFrame;

    // core class
    std::shared_ptr<KD_TREE<PointType>> m_pIkdTree = nullptr;
    std::shared_ptr<ImuProcess> m_pImu = nullptr;                   // imu process
    esekfom::esekf<state_ikfom, 12, input_ikfom> m_kf;              // esekf
};

} // ikd_odom
} // namespace lidar_odom
} // namespace humanoid_slam