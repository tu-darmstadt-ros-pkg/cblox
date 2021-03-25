cmake_minimum_required(VERSION 2.8.3)
project(cblox_ros)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED system)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
    src/submap_server.cc
  src/active_submap_visualizer.cc
  src/trajectory_visualizer.cc
    src/submap_conversions.cc

    src/cblox_factory.cpp
    src/generic_active_submap_visualizer.cc
    src/config_parser.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############

cs_add_executable(tsdf_submap_server
  src/tsdf_submap_server_node.cc
)
cs_add_executable(generic_submap_server
  src/generic_submap_server_node.cc
)
cs_add_executable(esdf_submap_server
  src/esdf_submap_server_node.cc
)
target_link_libraries(tsdf_submap_server ${PROJECT_NAME})
target_link_libraries(esdf_submap_server ${PROJECT_NAME})
target_link_libraries(generic_submap_server ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
