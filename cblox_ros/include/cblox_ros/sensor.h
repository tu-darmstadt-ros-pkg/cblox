#ifndef CBLOX_ROS_SENSOR_H_
#define CBLOX_ROS_SENSOR_H_

#include <string>
#include <thread>
#include <vector>

#include <cblox/core/common.h>
#include <cblox/core/generic_submap_collection.h>
#include <cblox/core/tsdf_submap.h>
#include <cblox/integrator/generic_submap_collection_integrator.h>
#include <cblox_ros/generic_active_submap_visualizer.h>
#include <ros/ros.h>
#include <voxblox_ros/transformer.h>

namespace cblox {

// TODO fix redefining/missing
// Default values for parameters
// constexpr bool kDefaultVerbose = true;
constexpr int kDefaultNumFramesPerSubmap_i = 20;
// constexpr double kDefaultMinTimeBetweenMsgsSec = 0.0;
// Defined in submap server

// Represents an abstract sensor for the generic submap server
// Behavior is adapted from the submap server
template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
class Sensor {
 public:
  explicit Sensor(  // std::shared_ptr<SubmapCollection<SubmapType>>
                    // submap_collection_ptr,
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      std::string world_frame,
      std::shared_ptr<GenericSubmapCollection<VoxelType>>
          submap_collection_ptr);

  virtual ~Sensor(){};

  // main callback, add and service queues like with clouds

  // add message to queue,
  void addMessageToQueue(const MsgType& msg);

  // service queue
  void serviceQueue();
  //->process message (handle queue)
  //->create new submap, if new submap was created, also handle submaps correct

  // process message and insert
  void processMessage(const MsgType& msg, const Transformation& T_G_C);
  //->use correct submap
  //->(integrator is used here), mabye use generic integrator
  //(check what could go wrong)
  //->use projection method

  bool getMessageFromQueue(std::queue<MsgType>* queue, MsgType* msg,
                           Transformation* T_G_C);

  // TODO
  bool mapInitialized() const { return map_initialized_; }
  bool newSubmapRequired() const;
  void createNewSubmap(const Transformation& T_G_C, const ros::Time& timestamp);
  void initializeMap(const Transformation& T_G_C);

  /*virtual void updateIntegratorSubmap(){
      static_cast<T*>(this)->updateIntegratorSubmap();
  }*/
  void integrateMessage(const MsgType& msg, const Transformation T_G_C) {
    static_cast<T*>(this)->integrateMessage(msg, T_G_C);
  }

  void finishSubmap(const SubmapID submap_id);

  // visualization (maybe do this in upper)/call this from upper/pass from here
  // to upper
  void register_visualizer(
      std::shared_ptr<
          GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>>
          visualizer);

  void resetIntegrator(typename GenericSubmapCollection<VoxelType>::Ptr submap_collection_ptr, typename IntegratorType::ConfigType integrator_config);

 protected:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool visualizer_registered_;
  std::shared_ptr<
      GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>>
      visualizer_ptr_;

  // msg delaying
  ros::Duration msg_delay_;   // TODO init
  ros::Time last_msg_stamp_;  // TODO init

  // queue
  std::queue<MsgType> msg_queue_;
  // TODO look at submap_queue

  voxblox::Transformer transformer_;

  std::string world_frame_;

  std::shared_ptr<GenericSubmapCollection<VoxelType>> submap_collection_ptr_;
  std::shared_ptr<GenericSubmapCollectionIntegrator<IntegratorType,
                                                    IntegrationData, VoxelType>>
      submap_collection_integrator_;

  // Number of frames integrated to the current submap
  int num_integrated_frames_current_submap_;
  // The number of frames integrated into a submap before requesting a new one.
  int num_integrated_frames_per_submap_;

  bool map_initialized_;
};
}  // namespace cblox

#endif  // CBLOX_ROS_SENSOR_H_

#include "cblox_ros/sensor_inl.h"
