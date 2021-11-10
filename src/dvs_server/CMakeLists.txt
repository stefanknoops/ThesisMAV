cmake_minimum_required(VERSION 2.8.3)
project(dvs_server)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(dvs_server
  src/server.cpp
  src/server_node.cpp
)

# make the nodelet into a library
cs_add_library(dvs_server_nodelet
  src/server_nodelet.cpp
  src/server.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(dvs_server_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES dvs_server_nodelet.xml nodelet_mono.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
