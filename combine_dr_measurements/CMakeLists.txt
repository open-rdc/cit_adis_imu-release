cmake_minimum_required(VERSION 2.8.3)
project(combine_dr_measurements)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES combine_dr_measurements
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odometry_publisher src/odometry_publisher.cpp)

target_link_libraries(odometry_publisher
    ${catkin_LIBRARIES}
)

