#ifndef CBLOX_ROS_SENSOR_INL_H_
#define CBLOX_ROS_SENSOR_INL_H_

namespace cblox {

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
       GeometryVoxelType, ColorVoxelType>::
    Sensor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
           std::string world_frame,
           std::shared_ptr<GenericSubmapCollection<VoxelType>>
               submap_collection_ptr)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      submap_collection_ptr_(submap_collection_ptr),
      transformer_(nh, nh_private),
      last_msg_stamp_(ros::Time(0)),
      msg_delay_(ros::Duration(1.0)),
      num_integrated_frames_current_submap_(0),
      num_integrated_frames_per_submap_(kDefaultNumFramesPerSubmap_i),
      visualizer_registered_(false) {
  submap_collection_integrator_.reset(
      new GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData,
                                            VoxelType>(submap_collection_ptr));
  // submap_collection_ptr_ = submap_collection_ptr;
  // submap_collection_integrator_.reset(
  //    new GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData,
  //    VoxelType, SubmapType> (submap_collection_ptr));
};

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType, ColorVoxelType>::addMessageToQueue(const MsgType&
                                                                      msg) {
  // TODO handle delay different
  msg_delay_ = ros::Duration(0.1);
  if (msg->header.stamp - last_msg_stamp_ > msg_delay_) {
    last_msg_stamp_ = msg->header.stamp;
    msg_queue_.push(msg);
  }
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType, ColorVoxelType>::serviceQueue() {
  MsgType msg;
  Transformation T_G_C;

  // drop old messages if queue is too long
  const size_t kMaxQueueSize = 10;
  while (msg_queue_.size() >= kMaxQueueSize) {
    msg_queue_.pop();
  }

  while (getMessageFromQueue(&msg_queue_, &msg, &T_G_C)) {
    // TODO somehow the correct submaps need to be determined
    processMessage(msg, T_G_C);

    // TODO this should be aligned to the existing behavior
    if (newSubmapRequired()) {
      createNewSubmap(T_G_C, msg->header.stamp);
    }
  }
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
bool Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType,
            ColorVoxelType>::getMessageFromQueue(std::queue<MsgType>* queue,
                                                 MsgType* msg,
                                                 Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *msg = queue->front();
  std::string frame_id = (*msg)->header.frame_id;

  // TODO remove this dirty workaround for a bag
  if (frame_id.compare("arm_thermal_cam") == 0) {
    std::cout << "problematic frame" << std::endl;
    frame_id = "arm_thermal_cam_frame";
  }
  if (transformer_.lookupTransform(frame_id, world_frame_, (*msg)->header.stamp,
                                   T_G_C)) {
    queue->pop();
    return true;
  } else {
    // std::cout << "couldn't lookup transform" << std::endl;
    if (queue->size() >= (kMaxQueueSize - 1)) {
      ROS_ERROR_THROTTLE(60,
                         "Input msg queue getting too long! Dropping "
                         "some msgs. Either unable to look up transform "
                         "timestamps or the processing is taking too long.");
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
      }
    }
  }
  return false;
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType,
            ColorVoxelType>::processMessage(const MsgType& msg,
                                            const Transformation& T_G_C) {
  // TODO handle if map is initialized /layer

  if (!mapInitialized()) {
    ROS_INFO("[CbloxServer] Initializing map.");
    initializeMap(T_G_C);
  }

  ros::WallTime start = ros::WallTime::now();
  integrateMessage(msg, T_G_C);
  ros::WallTime end = ros::WallTime::now();
  num_integrated_frames_current_submap_++;

  /*
  if (verbose_) {
ROS_INFO(
  "[CbloxServer] Finished integrating in %f seconds, have %lu blocks. "
  "%u frames integrated to current submap.",
  (end - start).toSec(),
  submap_collection_ptr_->getActiveTsdfMap()
      .getTsdfLayer()
      .getNumberOfAllocatedBlocks(),
  num_integrated_frames_current_submap_);
}
*/
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
bool Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType, ColorVoxelType>::newSubmapRequired() const {
  return (num_integrated_frames_current_submap_ >
          num_integrated_frames_per_submap_);
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType,
            ColorVoxelType>::createNewSubmap(const Transformation& T_G_C,
                                             const ros::Time& timestamp) {
  // finishing up the last submap
  if (!submap_collection_ptr_->empty()) {
    std::thread finish_submap_thread(
        &Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType,
                IntegrationData, GeometryVoxelType,
                ColorVoxelType>::finishSubmap,
        this, submap_collection_ptr_->getActiveSubmapID());
    finish_submap_thread.detach();
  }
  std::cout << "created submap" << std::endl;
  // Creating the submap
  const SubmapID submap_id = submap_collection_ptr_->createNewSubmap(T_G_C);
  // Activating the submap in the frame integrator
  // updateIntegratorSubmap();
  submap_collection_integrator_->switchToActiveSubmap();

  // Resetting current submap counters
  num_integrated_frames_current_submap_ = 0;

  // Updating the active submap mesher
  if (visualizer_registered_) visualizer_ptr_->switchToActiveSubmap();
  // TODO visualize

  // Publish the baseframes
  // visualizeSubmapBaseframes();
  // TODO visualize

  // Time the start of recording
  // submap_collection_ptr_->getActiveSubmapPtr()->startMappingTime(
  // timestamp.toNSec());
  // TODO mapping time

  // TODO verbose
  bool verbose_ = false;

  if (verbose_) {
    ROS_INFO_STREAM("Created a new submap with id: "
                    << submap_id << ". Total submap number: "
                    << submap_collection_ptr_->size());
  }
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType,
            ColorVoxelType>::initializeMap(const Transformation& T_G_C) {
  createNewSubmap(T_G_C, ros::Time::now());
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType, ColorVoxelType>::finishSubmap(const SubmapID
                                                                 submap_id) {
  if (submap_collection_ptr_->exists(submap_id)) {
    typename SubmapType::Ptr submap_ptr =
        submap_collection_ptr_->getSubmapPtr(submap_id);
    // Stopping the mapping interval.
    submap_ptr->stopMappingTime(ros::Time::now().toNSec());
    // Finish submap.
    submap_ptr->finishSubmap();
    // publishing the old submap
    // publishSubmap(submap_id);
    ROS_INFO("[CbloxServer] Finished submap %d", submap_id);
  }
  // TODO finish map
}

template <typename T, typename SubmapType, typename MsgType, typename VoxelType,
          typename IntegratorType, typename IntegrationData,
          typename GeometryVoxelType, typename ColorVoxelType>
void Sensor<T, SubmapType, MsgType, VoxelType, IntegratorType, IntegrationData,
            GeometryVoxelType, ColorVoxelType>::
    register_visualizer(
        std::shared_ptr<
            GenericActiveSubmapVisualizer<GeometryVoxelType, ColorVoxelType>>
            visualizer) {
  visualizer_ptr_ = visualizer;
  visualizer_registered_ = true;
}

/*template <typename SubmapType, typename MsgType, typename VoxelType, typename
IntegratorType, ty> void Sensor<SubmapType, MsgType, VoxelType,
IntegratorType>:: integrateMessage(const MsgType& msg, const Transformation
T_G_C) {
  //TODO
  //integrator_ptr_->integrate(T_G_C, msg);
  std::cout << "integrating " << std::endl;
}*/
}  // namespace cblox

#endif  // CBLOX_ROS_SENSOR_INL_H_