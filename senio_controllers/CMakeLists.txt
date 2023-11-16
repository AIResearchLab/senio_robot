cmake_minimum_required(VERSION 3.10.2)
project(senio_controllers)

find_package(catkin REQUIRED COMPONENTS
  # roscpp
  # rospy
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
