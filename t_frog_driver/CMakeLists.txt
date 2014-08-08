cmake_minimum_required(VERSION 2.8.3)
project(t_frog_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    hardware_interface
    controller_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES t_frog_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/usr/share/include)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(t_frog_driver_node src/t_frog_driver_node.cpp)

target_link_libraries(t_frog_driver_node
    /usr/local/lib/libypspur.a
    ${catkin_LIBRARIES}
)

