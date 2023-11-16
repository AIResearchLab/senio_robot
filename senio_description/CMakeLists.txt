cmake_minimum_required(VERSION 3.10.2)
project(senio_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package()
