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
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > dN;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > ddN;
    std::vector<double> N_double_vec;
    std::vector<double> R_double_vec;
    std::vector<double> dR_double_vec;
    std::vector<double> ddR_double_vec;
    // std::vector<double> ddN_double_vec;
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
        nav_msgs::Path Generate_NURBSCurve(Spline_Inf spline_inf, double t_intervel, std::string frame_id);
        nav_msgs::Path Generate_DerivativeBsplineCurve(Spline_Inf bspline_inf, int differential_times, double t_intervel, std::string frame_id);
        nav_msgs::Path Generate_DerivativeBasisFuncCurve(Spline_Inf bspline_inf, int differential_times, int index, double t_intervel, std::string frame_id);

        double CalculateCurvature(Spline_Inf spline_inf, double u_data, bool UsingNURBS);
        double CalculateCurvatureRadius(Spline_Inf spline_inf, double u_data, bool UsingNURBS);
        double CalculateCurveLength(Spline_Inf spline_inf, double start_u, double end_u, int sub_intervals, bool UsingNURBS);

        void ReadSplineInf(Spline_Inf *bspline_inf, int order, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point, std::vector<double> knot_vector);
        void ReadSplineInf(Spline_Inf *bspline_inf, std::vector<double> weight_vector);
        void ReadDiscreate2DPointFromLaunch(EigenTrajectoryPoint::Vector *input_point, std::vector<double> file_discreate_point);
        void ReadDiscreate2DPointFromLaunch(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *input_point, std::vector<double> file_discreate_point);
        void ShowDiscreatePoint(visualization_msgs::Marker *points, EigenTrajectoryPoint::Vector discreate_point);
        void ShowDiscreatePoint(visualization_msgs::Marker *points, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point);

        visualization_msgs::Marker ShowDiscreatePoint(EigenTrajectoryPoint::Vector& discreate_point, const std::string& frame_id, const std::string& name, double scale);
        visualization_msgs::Marker ShowDiscreatePoint(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point, const std::string& frame_id, const std::string& name, double scale);
    private:
        void CalculateDerivativeBasisFunc(Spline_Inf *spline_inf, double u_data, int differential_times);
        geometry_msgs::Point CalculateDerivativeCurvePoint(Spline_Inf *spline_inf, double u_data, int differential_times, bool UsingNURBS);
        
};

#endif //AGV_PATH_SMOOTHING_CURVE_COMMON_H_