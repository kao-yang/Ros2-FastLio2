#include "humanoid_slam/HumanoidSlamBuilder.h"

namespace humanoid_slam{

HumanoidSlamBuilder::HumanoidSlamBuilder( const std::string& sParamsFile ){
    if ( !LoadParams(sParamsFile) ){
        LOG(FATAL) << "load slam params fail, process shut down";
    }else{
        LOG(INFO) << "load slam parsms file: " << sParamsFile << " success";
    }
    
    if( m_slamConfig.nLidarOdomType == 0 ){
        LOG(INFO) << "lidar odom module close.";
    }else if ( m_slamConfig.nLidarOdomType == 1 ){
        LOG(INFO) << "lidar odom module: ikd_odom";
        m_pLidarOdom.reset( new  lidar_odom::ikd_odom::IkdOdomBuilder
                    ( m_sParamsDir + m_slamConfig.sLidarOdomParamsFile ));
        LOG(INFO) << "lidar odom module init over";
        m_vThreads.push_back( std::thread( &HumanoidSlamBuilder::ProcessLidarOdom, this ) );
        LOG(INFO) << "start lidar odom thread.";
    }

}

HumanoidSlamBuilder::~HumanoidSlamBuilder(){
    // Step 1. blocking queue clear
    m_qImu.Done();
    // Step 2. thread joint
    for( auto &thread : m_vThreads ){
        thread.join();
    }
}
void HumanoidSlamBuilder::AddSensorData( const std::string& sSensorId, 
                const sensor::ImuData& imuData ){
    LOG_FIRST_N(INFO,1) << "imu data name: " << sSensorId;
    // Step 1. pose extrapolator
    // TODO
    // Step 2. push to blocking queue for lidar odom
    m_qImu.Push(imuData);
}

void HumanoidSlamBuilder::AddSensorData( const std::string& sSensorId, 
                const sensor::TimedPointCloudData& cloudData ){
    LOG_FIRST_N(INFO,1) << "cloud data name: " << sSensorId;
    m_pLidarOdom->InputData(cloudData);
}

std::tuple< const sensor::TimedPointCloudData, const sensor::TimedPointCloudData,
            const sensor::PoseData > HumanoidSlamBuilder::GetLidarOdomData(){
    return std::make_tuple(std::move(m_pLidarOdom->GetUndistorCloud()), 
        std::move(m_pLidarOdom->GetLocalMap()), std::move(m_pLidarOdom->GetLocalPose()));
}

bool HumanoidSlamBuilder::LoadParams( const std::string& sParamsFile ){
    // Step 1. check file exist
    if ( !std::filesystem::exists(sParamsFile) ){
        LOG(ERROR) << "slam param file: " << sParamsFile << " not exits";
        return false;
    }

    // Step 2. load params from yaml file
    auto slamYaml = YAML::LoadFile( sParamsFile );
    try{
        m_slamConfig.nPoseExtrapolatorType = slamYaml["modules_type"]["pose_extrapolator"].as<int>();
        m_slamConfig.nLidarOdomType = slamYaml["modules_type"]["lidar_odom"].as<int>();
        m_slamConfig.nGridMapType = slamYaml["modules_type"]["grid_map"].as<int>();
        m_slamConfig.nLoopFusionType = slamYaml["modules_type"]["loop_fusion"].as<int>();
        m_slamConfig.nPoseGraphType = slamYaml["modules_type"]["pose_graph"].as<int>();

        m_slamConfig.sPoseExtrapolatorParamsFile 
            = slamYaml["modules_yaml"]["pose_extrapolator_yaml"].as<std::string>();
        m_slamConfig.sLidarOdomParamsFile = slamYaml["modules_yaml"]["lidar_odom_yaml"].as<std::string>();
        m_slamConfig.sGridMapParamsFile = slamYaml["modules_yaml"]["grid_map_yaml"].as<std::string>();
        m_slamConfig.sLoopFusionParamsFile = slamYaml["modules_yaml"]["loop_fusion_yaml"].as<std::string>();
        m_slamConfig.sPoseGraphParamsFile = slamYaml["modules_yaml"]["pose_graph_yaml"].as<std::string>();
    }
    catch(...){
        LOG(ERROR) << "load slam yaml file: " << sParamsFile << " fail";
        return false;
    }

    // Step 3.get parent dir
    std::filesystem::path filePath(sParamsFile);
    std::filesystem::path folderPath = filePath.parent_path();
    m_sParamsDir = folderPath.string();
    if (m_sParamsDir.back() != '/') {
        m_sParamsDir += '/';
    }
    // Step 4. log main params
    LOG(INFO) << "slam yaml main params: " << std::endl
            << "pose extrapolator type: " << m_slamConfig.nPoseExtrapolatorType << std::endl
            << "lidar odom type: " << m_slamConfig.nLidarOdomType << std::endl
            << "grid map type: " << m_slamConfig.nGridMapType << std::endl
            << "loop fusion type: " << m_slamConfig.nLoopFusionType << std::endl
            << "pose graph type: " << m_slamConfig.nPoseGraphType << std::endl;

    return true;
}


void HumanoidSlamBuilder::ProcessLidarOdom(){
    while(1){
        auto curImuData = m_qImu.Pop();
        if( m_qImu.IsDone() ){ break; }
        m_pLidarOdom->InputData( curImuData );
        m_pLidarOdom->RunOdom();
    }
}

}