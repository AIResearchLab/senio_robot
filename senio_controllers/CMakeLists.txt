cmake_minimum_required(VERSION 3.10.2)
project(senio_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  kdl_parser
  realtime_tools
  controller_interface
  cartesian_trajectory_controller
  cartesian_trajectory_interpolation
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    kdl_parser
    realtime_tools
    controller_interface
    cartesian_trajectory_controller
    cartesian_trajectory_interpolation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/arthropod_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
