cmake_minimum_required(VERSION 3.10.2)
project(senio_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  StaticPose.action
)

add_message_files(
  FILES
  Dynamixel.msg
  Leg.msg
  SENCE.msg
  Target_Buffer.msg
  Target.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)
