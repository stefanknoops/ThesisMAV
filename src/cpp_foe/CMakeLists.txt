cmake_minimum_required(VERSION 3.0.2)
project(cpp_foe)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dvs_of_msg
  message_generation
)

add_message_files(
  FILES
  FoE.msg
)

# build the header files from the msg files, and notify catkin about the dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cpp_foe
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(cpp_foe 
   src/cpp_foe.cpp
   src/signum.cpp
)

target_link_libraries(cpp_foe ${catkin_LIBRARIES})


