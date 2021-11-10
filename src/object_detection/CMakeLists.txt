cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dvs_of_msg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detection
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection 
   src/object_detection.cpp
   src/prepEstimateClustersCPP.cpp
   src/kdtree.cpp
   src/EstimateClusterTTC.cpp
   src/CollisionCheck.cpp
   src/controller.cpp
)

target_link_libraries(object_detection ${catkin_LIBRARIES})

