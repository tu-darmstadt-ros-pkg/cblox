#ifndef CBLOX_ROS_POSE_GRAPH_UPDATER_H_
#define CBLOX_ROS_POSE_GRAPH_UPDATER_H_

#include <cartographer_ros_msgs/StampedSubmapEntry.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <cblox_ros/map_history.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>

#include <cblox_ros/definitions.h>

namespace cblox {

class PoseGraphUpdater {
    public:

    typedef std::shared_ptr<PoseGraphUpdater> Ptr;
    explicit PoseGraphUpdater(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const std::string submap_anouncement_topic,
                                       const std::string submap_list_topic, 
                                       const double min_remesh_angle, 
                                       const double min_remesh_distance);
    virtual ~PoseGraphUpdater() {}

    //function to add maps 
    void initMaps(std::shared_ptr<MapVariantsMap> maps);

    //funtion to add sensors
    void initSensors(std::shared_ptr<SensorVariantsMap> sensors);

    //function to add visualizers
    void initVisualizers(std::shared_ptr<VisualizerVariantsMap> visualizers);

    //callbacks
    void submapAnouncementCallback(const cartographer_ros_msgs::StampedSubmapEntry::Ptr& msg);
    void submapListCallback(const cartographer_ros_msgs::SubmapList::Ptr& msg);

    protected:

    
    //NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //Subscriber
    ros::Subscriber submap_anouncement_sub_;
    ros::Subscriber submap_list_sub_;

    //list of maps
    //list of sensors
    //list of visualizers
    std::shared_ptr<MapVariantsMap> maps_;
    std::shared_ptr<SensorVariantsMap> sensors_;
    std::shared_ptr<VisualizerVariantsMap> visualizers_;

    //remeshing
    double min_remesh_angle_;
    double min_remesh_distance_;

    std::shared_ptr<MapHistory> map_history_;
};

class init_sensor_visitor : public boost::static_visitor<> {
 public:
  template <typename T>
  void operator()(T& op, std::shared_ptr<MapHistory>& map_history) const {
    op->set_pose_graph_mode(map_history);
  }
};
}  // namespace cblox

#endif // CBLOX_ROS_POSE_GRAPH_UPDATER_H_

#include "cblox_ros/pose_graph_updater_inl.h"