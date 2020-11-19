#ifndef CBLOX_ROS_METHOD_TOOLBOX_H_
#define CBLOX_ROS_METHOD_TOOLBOX_H_

#include <cblox/integrator/projection_integrator.h>

namespace cblox {
class MethodToolbox {
  template <typename T>
  static ProjectionData<T> createProjectionData();
}
}  // namespace cblox

#endif  // CBLOX_ROS_METHOD_TOOLBOX_H_