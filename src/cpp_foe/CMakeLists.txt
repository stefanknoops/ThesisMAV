cmake_minimum_required(VERSION 3.0.2)
project(cpp_foe)

find_package(catkin_simple REQUIRED)

catkin_simple()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  std_msgs
#  dvs_of_msg
#  message_generation
#)

#add_message_files(
#  FILES
#  FoE.msg
#)

# build the header files from the msg files, and notify catkin about the dependencies
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)


#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cpp_foe
#)


#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)



cs_add_executable(cpp_foe 
   src/cpp_foe.cpp
   src/signum.cpp
)

# make the executable for the visualization node
cs_add_executable(foe_visualization
  src/foe_visualization.cpp
  src/foe_visualization_node.cpp
)

target_link_libraries(foe_visualization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS foe_visualization
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(cpp_foe ${catkin_LIBRARIES})

cs_install()
cs_export()