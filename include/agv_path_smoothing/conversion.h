#ifndef AGV_PATH_SMOOTHING_CONVERSION_H_
#define AGV_PATH_SMOOTHING_CONVERSION_H_

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

inline Eigen::Vector3d EigenVecter3dFromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

#endif //AGV_PATH_SMOOTHING_CONVERSION_H_