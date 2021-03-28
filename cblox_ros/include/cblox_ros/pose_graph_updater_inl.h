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

            map_history_ = std::make_shared<MapHistory>();
        }

    //function to add maps
    void PoseGraphUpdater::initMaps(std::shared_ptr<MapVariantsMap> maps) {
        maps_ = maps;
        std::cout << "inited maps" << std::endl;
        //TODO in this functions update maps etc. to pose_graph mode

        // for (auto it = maps_->begin(); it != maps_->end(); it++) {
        //    std::cout << it->first << std::endl;
        // it.second.set_posegraph_mode();
        //}
    }

    //funtion to add sensors
    void PoseGraphUpdater::initSensors(std::shared_ptr<SensorVariantsMap> sensors) {
        sensors_ = sensors;
        std::cout << "inited sensors" << std::endl;

        auto bound_visitor = std::bind(init_sensor_visitor(),
                                       std::placeholders::_1, map_history_);

        for (auto it = sensors_->begin(); it != sensors_->end(); it++) {
          // std::cout << it->first << std::endl;

          // it->second.set_posegraph_mode(map_history_);
          boost::apply_visitor(bound_visitor, it->second);
        }
    }

    //function to add visualizers
    void PoseGraphUpdater::initVisualizers(std::shared_ptr<VisualizerVariantsMap> visualizers) {
        visualizers_ = visualizers;
        std::cout << "inited visualizers" << std::endl;

        // for (auto it = visualizers_->begin(); it != visualizers_->end();
        // it++) {
        //    std::cout << it->first << std::endl;
        // it.second.set_posegraph_mode();
        //}
    }

    //callbacks
    void PoseGraphUpdater::submapAnouncementCallback(const cartographer_ros_msgs::StampedSubmapEntry::Ptr& msg) {
        std::cout << "anouncement callback" << std::endl;
        std::cout << "creating new submaps" << std::endl;
        // TODO create submap
        // call finish submap of submap and then create new
        // it.second.createNewSubmap();
        // take transform, stamp and id from map
        // check that childmap creation still works

        // Do this over sensors instead (maybe lock mutex for map)
        // so that childmaps are done correctly
        // keep submap creation history as maps. check in sensor, if submap is
        // up to date (last entry of map) else create new [done to not
        // completely change behaviour]

        // TODO drop old messages??
        MapHistoryEntry entry;
        entry.id = msg->submap.submap_index;
        Transformation transform;
        tf::Transform tf_trans;
        tf::poseMsgToTF(msg->submap.pose, tf_trans);
        tf::transformTFToKindr(tf_trans, &transform);
        entry.T_w_m = transform;
        entry.stamp = msg->submap.start_stamp;
        map_history_->push_back(entry);
    }
    void PoseGraphUpdater::submapListCallback(const cartographer_ros_msgs::SubmapList::Ptr& msg) {
      // TODO enable dropping of invalid messages (here and maybe also on
      // anouncement)
      std::cout << "list callback" << std::endl;
    }

}  // namespace cblox

#endif  // CBLOX_ROS_POSE_GRAPH_UPDATER_INL_H_