cmake_minimum_required(VERSION 3.0.2)
project(crowd_navigation_msgs)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
    CrowdMotionPrediction.msg
    CrowdMotionPredictionStamped.msg
    MotionPrediction.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crowd_navigation_msgs
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
#  DEPENDS system_lib
)