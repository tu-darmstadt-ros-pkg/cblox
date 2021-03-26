#ifndef CBLOX_ROS_DEFINITIONS_H_
#define CBLOX_ROS_DEFINITIONS_H_


#include <boost/variant.hpp>
#include <cblox/core/generic_submap_collection.h>
#include <cblox/core/voxel.h>
#include <voxblox/core/voxel.h>
#include <cblox_ros/lidar_sensor.h>
#include <cblox_ros/rgb_sensor.h>
#include <cblox_ros/thermal_sensor.h>
#include <cblox_ros/generic_active_submap_visualizer.h>

namespace cblox {

typedef boost::variant<
    typename GenericSubmapCollection<voxblox::TsdfVoxel>::Ptr,
    typename GenericSubmapCollection<voxblox::RGBVoxel>::Ptr,
    typename GenericSubmapCollection<voxblox::IntensityVoxel>::Ptr>
    SubmapCollectionVariants;

typedef boost::variant<
    typename LIDARSensor<GenericSubmap<voxblox::TsdfVoxel>,
                         voxblox::TsdfVoxel>::Ptr,
    typename RGBSensor<GenericSubmap<voxblox::RGBVoxel>,
                       voxblox::TsdfVoxel>::Ptr,
    typename ThermalSensor<GenericSubmap<voxblox::IntensityVoxel>,
                           voxblox::TsdfVoxel>::Ptr>
    SensorVariants;

typedef boost::variant<typename GenericActiveSubmapVisualizer<
                           voxblox::TsdfVoxel, voxblox::TsdfVoxel>::Ptr,
                       typename GenericActiveSubmapVisualizer<
                           voxblox::TsdfVoxel, voxblox::RGBVoxel>::Ptr,
                       typename GenericActiveSubmapVisualizer<
                           voxblox::TsdfVoxel, voxblox::IntensityVoxel>::Ptr>
    VisualizerVariants;

typedef std::map<std::string, SubmapCollectionVariants> MapVariantsMap;
typedef std::map<std::string, SensorVariants> SensorVariantsMap;
typedef std::map<std::string, VisualizerVariants> VisualizerVariantsMap;
}

#endif  //CBLOX_ROS_DEFINITIONS_H_