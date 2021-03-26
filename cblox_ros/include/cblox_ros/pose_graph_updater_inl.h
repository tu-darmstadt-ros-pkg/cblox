#ifndef CBLOX_ROS_POSE_GRAPH_UPDATER_INL_H_
#define CBLOX_ROS_POSE_GRAPH_UPDATER_INL_H_

namespace cblox {

    PoseGraphUpdater::PoseGraphUpdater(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private,
                                       const std::string submap_anouncement_topic,
                                       const std::string submap_list_topic,
                                       const double min_remesh_angle,
                                       const double min_remesh_distance)
        : nh_(nh), nh_private_(nh_private), min_remesh_angle_(min_remesh_angle), min_remesh_distance_(min_remesh_distance) {
            submap_anouncement_sub_ = nh_.subscribe(submap_anouncement_topic, 1000, &PoseGraphUpdater::submapAnouncementCallback, this);
            submap_list_sub_ = nh_.subscribe(submap_list_topic, 10, &PoseGraphUpdater::submapListCallback, this);
        }

    //function to add maps
    void PoseGraphUpdater::initMaps(std::shared_ptr<MapVariantsMap> maps) {
        maps_ = maps;
        std::cout << "inited maps" << std::endl;
        //TODO in this functions update maps etc. to pose_graph mode
    }

    //funtion to add sensors
    void PoseGraphUpdater::initSensors(std::shared_ptr<SensorVariantsMap> sensors) {
        sensors_ = sensors;
        std::cout << "inited sensors" << std::endl;
    }

    //function to add visualizers
    void PoseGraphUpdater::initVisualizers(std::shared_ptr<VisualizerVariantsMap> visualizers) {
        visualizers_ = visualizers;
        std::cout << "inited visualizers" << std::endl;
    }

    //callbacks
    void PoseGraphUpdater::submapAnouncementCallback(const cartographer_ros_msgs::StampedSubmapEntry::Ptr& msg) {
        std::cout << "anouncement callback" << std::endl;
    }
    void PoseGraphUpdater::submapListCallback(const cartographer_ros_msgs::SubmapList::Ptr& msg) {
        std::cout << "list callback" << std::endl;
    }

}  // namespace cblox

#endif  // CBLOX_ROS_POSE_GRAPH_UPDATER_INL_H_