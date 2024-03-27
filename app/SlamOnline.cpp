#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "laser_mapping.h"
// gflag
DEFINE_string(config_dir, "rs16/", "path to config dir");

int main(int argc, char * argv[])
{
    // Step 1. init
    rclcpp::init(argc, argv);                           // ros2
    
    // Keep going if an unknown flag is encountered, 
    // because there is a crash with ros2 launch --ros-args -r
    // https://github.com/gflags/gflags/issues/148#issuecomment-318826625
    google::AllowCommandLineReparsing();
    gflags::ParseCommandLineFlags(&argc, &argv, false);  // gflag

    FLAGS_stderrthreshold = google::INFO;               // glog
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    // Step 2. get params
    std::string sConfigParamDir = std::string(ROOT_DIR) 
                    + "config/" + FLAGS_config_dir;
    if ( !sConfigParamDir.empty() && sConfigParamDir.back()!='/' ){
        sConfigParamDir += "/";
    }

    LOG(INFO) << "config dir from gflag: " << sConfigParamDir;

    // Step 3. shut down
    rclcpp::shutdown();

    return 0;
}