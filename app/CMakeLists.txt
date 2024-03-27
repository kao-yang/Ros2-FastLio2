# add exe
add_executable(
  slam_online 
  SlamOnline.cpp
  )

target_link_libraries(
  slam_online
  gflags
  ${PROJECT_NAME}
  )

ament_target_dependencies(
  slam_online 
  rclcpp 
  std_msgs 
  sensor_msgs
  nav_msgs
  )

install(TARGETS
  slam_online
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch
  # rviz
  DESTINATION share/${PROJECT_NAME})