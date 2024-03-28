#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace humanoid_slam{
namespace sensor{

struct TimedPointCloudData{
    double fBeginTime = -1;
    double fEndTime = -1;
    CloudPtr pPointCloud = CloudPtr(new PointCloudType);
};

} // namespace sensor
} // namespace humanoid_slam