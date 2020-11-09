cmake_minimum_required(VERSION 2.8.0)
project(cblox)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Mark these as system deps to supress warnings.
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

############
# PROTOBUF #
############
# Taken from voxblox
set(PROTO_DEFNS proto/cblox/QuatTransformation.proto
                proto/cblox/Submap.proto
                proto/cblox/SubmapCollection.proto
                proto/cblox/GenericSubmapCollection.proto)
set(BASE_PATH "proto")
set(PROTOBUF_COMPILE_PYTHON true)

PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)

cs_add_library(${PROJECT_NAME}_lib
  src/core/tsdf_submap.cpp
  src/core/tsdf_esdf_submap.cpp
  src/integrator/tsdf_submap_collection_integrator.cpp
  src/utils/quat_transformation_protobuf_utils.cpp
  src/mesh/submap_mesher.cpp
  src/io/transformation_io.cpp
  

  #either explicitly instantiate here, or make to header? Right now instantiating
  #TODO think about this
  src/core/voxel.cpp
  src/core/generic_submap.cpp
  src/integrator/generic_submap_collection_integrator.cpp
  #src/integrator/projection_integrator.cpp
  src/integrator/rgb_projection_integrator.cpp
  src/integrator/thermal_projection_integrator.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_proto)

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
