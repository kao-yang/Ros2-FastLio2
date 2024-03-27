#include "laser_mapping.h"

namespace fast_lio {

LaserMapping::LaserMapping(const std::string& sParamsDir)
        : Node("ros2_fast_lio2")
{
        // Step 0. init ros2 node handle
    node_handler_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    // Step 1.init ptr
    // TODO

    // Step 2.load params
    // if ( !LoadParamsFromYAML( sParamsDir+ "ikdodom.yaml") ){
    //     LOG(FATAL) << "load param fail,process exit";
    // }else{
    //     LOG(INFO) << "load param success";
    // }

}


} // namespace fast_lio