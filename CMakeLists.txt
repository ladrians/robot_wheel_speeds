cmake_minimum_required(VERSION 2.8.3)
project(robot_wheel_speeds)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(FILES  WheelVelocities.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_wheel_speeds
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS})

FILE(GLOB srcFiles "src/*.cpp")
link_directories(/usr/local/include)
add_executable(robot_wheel_speeds ${srcFiles})
target_link_libraries(robot_wheel_speeds ${catkin_LIBRARIES} wiringPi -lrt)
