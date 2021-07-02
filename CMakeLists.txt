cmake_minimum_required(VERSION 2.8.3)
project(tucan_ros_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
)

find_package(cmake_modules REQUIRED)

#generate_dynamic_reconfigure_options(
#    )
    
#catkin_package(
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

