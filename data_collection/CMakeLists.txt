cmake_minimum_required(VERSION 3.5)

project(data_collection)

# Find the required packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_filters
        laser_geometry
        pcl_conversions
        pcl_ros
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(fuse_sensors_to_point_cloud
        src/fuse_sensors_to_point_cloud.cpp
        )

target_link_libraries(fuse_sensors_to_point_cloud
        ${catkin_LIBRARIES}
        )