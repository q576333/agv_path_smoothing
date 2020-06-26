#ifndef AGV_PATH_SMOOTHING_CONVERSION_H_
#define AGV_PATH_SMOOTHING_CONVERSION_H_
#include "agv_path_smoothing/Curve_common.h"

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

inline Eigen::Vector3d EigenVecter3dFromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline EigenTrajectoryPoint::Vector EigenTrajectoryVectorFromVector(const std::vector<geometry_msgs::PoseStamped> &plan)
{
  EigenTrajectoryPoint::Vector eigen_trajectory_point_vec;
  EigenTrajectoryPoint eigen_trajectory_point;

  for(int i = 0; i < plan.size(); i++)
  {
    eigen_trajectory_point.position(0) = plan.at(i).pose.position.x;
    eigen_trajectory_point.position(1) = plan.at(i).pose.position.y;
    eigen_trajectory_point_vec.push_back(eigen_trajectory_point);
  }

  return eigen_trajectory_point_vec;
}

inline EigenTrajectoryPoint::Vector EigenTrajectoryVectorFromVector(const std::vector<geometry_msgs::Point> &discreate_point_vec)
{
  EigenTrajectoryPoint::Vector eigen_trajectory_point_vec;
  EigenTrajectoryPoint eigen_trajectory_point;

  for(int i = 0; i < discreate_point_vec.size(); i++)
  {
    eigen_trajectory_point.position(0) = discreate_point_vec.at(i).x;
    eigen_trajectory_point.position(1) = discreate_point_vec.at(i).y;
    eigen_trajectory_point_vec.push_back(eigen_trajectory_point);
  }
  
  return eigen_trajectory_point_vec;
}

#endif //AGV_PATH_SMOOTHING_CONVERSION_H_