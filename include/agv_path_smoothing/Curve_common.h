#ifndef AGV_PATH_SMOOTHING_CURVE_COMMON_H_
#define AGV_PATH_SMOOTHING_CURVE_COMMON_H_

#include <Eigen/Eigen>
//#include <vector>
// #include <deque>
// #include <iostream>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

struct Spline_Inf
{
    int order;
    std::vector<double> knot_vector;
    std::vector<double> weight;
    //std::vector<double> N;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > N;
    //Eigen::Matrix<Eigen::VectorXd, Eigen::Dynamic, Eigen::Dynamic> N; //bsaic function
    Eigen::MatrixXd curvefit_N;
    
};

struct EigenTrajectoryPoint
{
    typedef std::vector<EigenTrajectoryPoint, Eigen::aligned_allocator<EigenTrajectoryPoint> > Vector;
    double u_data;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    //Eigen::VectorXd u_data;
};

class Curve_common
{
    public:
        Curve_common();
        nav_msgs::Path Generate_Line(geometry_msgs::Point start_point, geometry_msgs::Point end_point, double t_intervel, std::string frame_id);
        nav_msgs::Path Generate_BezierCurve(EigenTrajectoryPoint::Vector control_point, double t_intervel, std::string frame_id);
        nav_msgs::Path Generate_BsplineCurve(Spline_Inf bspline_inf, double t_intervel, std::string frame_id);
        void ReadSplineInf(Spline_Inf *bspline_inf, int order, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point, std::vector<double> knot_vector);
        void ReadDiscreate2DPointFromLaunch(EigenTrajectoryPoint::Vector *input_point, std::vector<double> file_discreate_point);
        void ReadDiscreate2DPointFromLaunch(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *input_point, std::vector<double> file_discreate_point);
        void ShowDiscreatePoint(visualization_msgs::Marker *points, EigenTrajectoryPoint::Vector discreate_point);
        void ShowDiscreatePoint(visualization_msgs::Marker *points, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point);
        visualization_msgs::Marker ShowDiscreatePoint(EigenTrajectoryPoint::Vector& discreate_point, const std::string& frame_id, const std::string& name, double scale);
        visualization_msgs::Marker ShowDiscreatePoint(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point, const std::string& frame_id, const std::string& name, double scale);
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