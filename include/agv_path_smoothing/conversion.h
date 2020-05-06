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
  double x = 0;
  double y = 0;
  EigenTrajectoryPoint::Vector eigen_trajectory_point_vec;
  
  EigenTrajectoryPoint eigen_trajectory_point;
  Eigen::Vector3d point_vector;
  //eigen_trajectory_point_vec.resize(plan.size());
  std::cout << "size of plan vector :" << plan.size() << "\n";
  for(int i = 0; i < plan.size(); i++)
  {
    x = plan.at(i).pose.position.x;
    y = plan.at(i).pose.position.y;
    point_vector(0) = x;
    point_vector(1) = y;
    eigen_trajectory_point.position = point_vector;
    eigen_trajectory_point_vec.push_back(eigen_trajectory_point);
  }
  // for(int i = 0; i < eigen_trajectory_point_vec.size(); i++)
  // {
  //     std::cout << i << "'s eigen_trajectory_point_vec in path smoothing x : " << eigen_trajectory_point_vec.at(i).position(0) << "\n";
  //     std::cout << i << "'s eigen_trajectory_point_vec in path smoothing y : " << eigen_trajectory_point_vec.at(i).position(1) << "\n";
  // }
  return eigen_trajectory_point_vec;
}

#endif //AGV_PATH_SMOOTHING_CONVERSION_H_