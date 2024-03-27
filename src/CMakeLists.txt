add_library(${PROJECT_NAME}
    laser_mapping.cpp
)

target_link_libraries(${PROJECT_NAME}
    glog 
    yaml-cpp
    tbb
    ${PCL_LIBRARIES}
)