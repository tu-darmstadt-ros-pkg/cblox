#include <glog/logging.h>
#include <ros/ros.h>

#include <cblox_ros/generic_submap_server.h>
//#include <cblox_ros/rgb_sensor.h>
#include <cblox/core/submap_collection.h>
#include <cblox/core/tsdf_submap.h>
//#include <cblox_ros/rgb_sensor.h>
#include <voxblox/core/voxel.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cblox::GenericSubmapServer node(nh, nh_private);

  std::cout << "started generic submap server" << std::endl;
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}
