#ifndef CBLOX_ROS_MAP_HISTORY_H_
#define CBLOX_ROS_MAP_HISTORY_H_

#include <vector>
#include <ros/ros.h>

#include <cblox/core/common.h>

namespace cblox {

struct MapHistoryEntry {
    int id;
    Transformation T_w_m;
    ros::Time stamp;
};

typedef std::vector<MapHistoryEntry> MapHistory;

}

#endif  // CBLOX_ROS_MAP_HISTORY_H_