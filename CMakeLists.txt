cmake_minimum_required(VERSION 2.8.3)
project(ros2d2)

add_compile_options(-std=c++11 -fpermissive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS src
#  LIBRARIES ros2d2
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros2d2_voice src/ros2d2_voice.cpp)
target_link_libraries(ros2d2_voice
  ${catkin_LIBRARIES}
  asound
)

