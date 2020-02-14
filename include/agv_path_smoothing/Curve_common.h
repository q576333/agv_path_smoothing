#ifndef AGV_PATH_SMOOTHING_CURVE_COMMON_H_
#define AGV_PATH_SMOOTHING_CURVE_COMMON_H_

#include <Eigen/Eigen>
// #include <deque>
// #include <iostream>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

struct EigenTrajectoryPoint
{
    typedef std::vector<EigenTrajectoryPoint, Eigen::aligned_allocator<EigenTrajectoryPoint> > Vector;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
};

class Curve_common
{
    public:
        Curve_common();
        nav_msgs::Path Generate_Line(geometry_msgs::Point start_point, geometry_msgs::Point end_point, double t_intervel);
        nav_msgs::Path Generate_BezierCurve(EigenTrajectoryPoint::Vector control_point, double t_intervel);
        void ReadDiscreate2DPointFromLaunch(EigenTrajectoryPoint::Vector *input_point, std::vector<double> file_discreate_point);
        void ShowDiscreatePoint(visualization_msgs::Marker *points, EigenTrajectoryPoint::Vector discreate_point);
    private:
};

// #define MAV_MSGS_CONCATENATE(x, y) x##y
// #define MAV_MSGS_CONCATENATE2(x, y) MAV_MSGS_CONCATENATE(x, y)
// #define MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE)                    \
//   typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>> \
//       MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Vector);                        \
//   typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>>  \
//       MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Deque);

// MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPoint)
#endif //AGV_PATH_SMOOTHING_CURVE_COMMON_H_