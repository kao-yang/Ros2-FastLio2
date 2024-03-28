#pragma once

#include <filesystem>
#include <vector>
#include <thread>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "humanoid_slam/HumanoidSlamInterface.h"
#include "humanoid_slam/BlockingQueue.h"
#include "lidar_odom/LidarOdomInterface.h"
#include "lidar_odom/ikd_odom/IkdOdomBuilder.h"

namespace humanoid_slam{

struct SlamConfig{
    int nPoseExtrapolatorType;
    int nLidarOdomType;
    int nGridMapType;
    int nLoopFusionType;
    int nPoseGraphType;

    std::string sPoseExtrapolatorParamsFile;
    std::string sLidarOdomParamsFile;
    std::string sGridMapParamsFile;
    std::string sLoopFusionParamsFile;
    std::string sPoseGraphParamsFile;
};

class HumanoidSlamBuilder : public HumanoidSlamInterface{
public:

    HumanoidSlamBuilder( const std::string& sParamsFile );
    ~ HumanoidSlamBuilder()override;
    HumanoidSlamBuilder( const HumanoidSlamBuilder& ) = delete;
    HumanoidSlamBuilder& operator=( const HumanoidSlamBuilder& ) = delete;

    void AddSensorData( const std::string& sSensorId, 
                const sensor::ImuData& imuData ) override;

    void AddSensorData( const std::string& sSensorId, 
                const sensor::TimedPointCloudData& cloudData ) override;

    std::tuple< const sensor::TimedPointCloudData, const sensor::TimedPointCloudData,
                        const sensor::PoseData > GetLidarOdomData() override;
    
    bool LoadParams( const std::string& sParamsFile );

    void ProcessLidarOdom();

private:

    SlamConfig m_slamConfig;
    std::string m_sParamsDir;
    std::vector<std::thread> m_vThreads;
    std::unique_ptr<lidar_odom::LidarOdomInterface> m_pLidarOdom = nullptr;
    BlockingQueue< sensor::ImuData > m_qImu;
};

}//humanoid_slam