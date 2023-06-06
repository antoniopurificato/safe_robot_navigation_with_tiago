cmake_minimum_required(VERSION 3.0.2)
project(crowd_navigation)
find_package(catkin REQUIRED
  crowd_navigation_core
  crowd_navigation_gazebo
  crowd_navigation_msgs
)
catkin_metapackage()