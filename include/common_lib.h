#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <unsupported/Eigen/ArpackSupport>

#include "options.h"
#include "so3_math.h"

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace fast_lio::common {

constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

inline std::string DEBUG_FILE_DIR(const std::string &name) { return std::string(ROOT_DIR) + "Log/" + name; }

// using Pose6D = faster_lio::Pose6D;
using V3D = Eigen::Vector3d;
using V4D = Eigen::Vector4d;
using V5D = Eigen::Matrix<double, 5, 1>;
using M3D = Eigen::Matrix3d;
using M4D = Eigen::Matrix4d;
using V3F = Eigen::Vector3f;
using V4F = Eigen::Vector4f;
using V5F = Eigen::Matrix<float, 5, 1>;
using M3F = Eigen::Matrix3f;
using M4F = Eigen::Matrix4f;

using VV3D = std::vector<V3D, Eigen::aligned_allocator<V3D>>;
using VV3F = std::vector<V3F, Eigen::aligned_allocator<V3F>>;
using VV4F = std::vector<V4F, Eigen::aligned_allocator<V4F>>;
using VV4D = std::vector<V4D, Eigen::aligned_allocator<V4D>>;
using VV5F = std::vector<V5F, Eigen::aligned_allocator<V5F>>;
using VV5D = std::vector<V5D, Eigen::aligned_allocator<V5D>>;

const M3D Eye3d = M3D::Identity();
const M3F Eye3f = M3F::Identity();
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

/// sync imu and lidar measurements
struct MeasureGroup {
    MeasureGroup() { this->lidar_.reset(new PointCloudType()); };

    double lidar_bag_time_ = 0;
    double lidar_end_time_ = 0;
    PointCloudType::Ptr lidar_ = nullptr;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_;
};

template <typename T>
T rad2deg(const T &radians) {
    return radians * 180.0 / M_PI;
}

template <typename T>
T deg2rad(const T &degrees) {
    return degrees * M_PI / 180.0;
}

struct Pose6D{
    double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
    V3D acc;      // the preintegrated total acceleration (global frame) at the Lidar origin
    V3D gyr;      // the unbiased angular velocity (body frame) at the Lidar origin
    V3D vel;      // the preintegrated velocity (global frame) at the Lidar origin
    V3D pos;      // the preintegrated position (global frame) at the Lidar origin
    M3D rot;      // the preintegrated rotation (global frame) at the Lidar origin 9
};

/**
 * set a pose 6d from ekf status
 * @tparam T
 * @param t
 * @param a
 * @param g
 * @param v
 * @param p
 * @param R
 * @return
 */
template <typename T>
Pose6D set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                  const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R) {
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++) {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
    }
    rot_kp.rot = R;
    return rot_kp;
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec_:  normalized x0
*/
/**
 * 计算一组点的法线
 * @tparam T
 * @param normvec
 * @param point
 * @param threshold
 * @param point_num
 * @return
 */
template <typename T>
bool esti_normvector(Eigen::Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold,
                     const int &point_num) {
    Eigen::MatrixXf A(point_num, 3);
    Eigen::MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++) {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++) {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold) {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

/**
 * squared distance
 * @param p1
 * @param p2
 * @return
 */
inline float calc_dist(const PointType &p1, const PointType &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }

/**
 * estimate a plane
 * @tparam T
 * @param pca_result
 * @param point
 * @param threshold
 * @return
 */
template <typename T>
inline bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold = 0.1f) {
    if (point.size() < options::MIN_NUM_MATCH_POINTS) {     // 点数量足够多
        return false;
    }

    Eigen::Matrix<T, 3, 1> normvec;

    // 求法线
    if (point.size() == options::NUM_MATCH_POINTS) {        // 点数量刚好满足阈值
        Eigen::Matrix<T, options::NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, options::NUM_MATCH_POINTS, 1> b;

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < options::NUM_MATCH_POINTS; j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        normvec = A.colPivHouseholderQr().solve(b);
    } else {                                                // 点数量多于阈值
        Eigen::MatrixXd A(point.size(), 3);
        Eigen::VectorXd b(point.size(), 1);

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point.size(); j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
        normvec(0, 0) = n(0, 0);
        normvec(1, 0) = n(1, 0);
        normvec(2, 0) = n(2, 0);
    }

    T n = normvec.norm();   // 法线模长
    pca_result(0) = normvec(0) / n;     // 单位法向量
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;            // ?

    for (const auto &p : point) {
        Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
        temp[3] = 1.0;
        if (fabs(pca_result.dot(temp)) > threshold) {
            return false;
        }
    }
    return true;
}
inline double FromRosTime(const builtin_interfaces::msg::Time& time){
    return time.sec + static_cast<double>(time.nanosec) / 1000000000.f;
}

inline builtin_interfaces::msg::Time ToRosTime(const double& time){
    builtin_interfaces::msg::Time stamp;
    int32_t sec = static_cast<int32_t>(time);
    int32_t nanosec = static_cast<int32_t>( (time - sec) * 1000000000 );
    stamp.sec = sec;
    stamp.nanosec = nanosec;
    return stamp;
}

}  // namespace fast_lio::common