#include <ros/ros.h>

#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/conversion.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer_spline");
    ros::NodeHandle private_nh("~");

    double t_intervel = 0.01;

    ros::Publisher pub = private_nh.advertise<nav_msgs::Path>("Curve_result", 10);
    ros::Publisher pub_derivative_curve = private_nh.advertise<nav_msgs::Path>("Derivative_curve_result", 10);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10);

    nav_msgs::Path myCurve;
    nav_msgs::Path derivative_myCurve;
    visualization_msgs::Marker points;
    std::string frame_id = "odom";
    std::vector<double> input_control_point;
    std::vector<double> input_knot_vector;
    std::vector<double> weight_vector;
    Spline_Inf input_spline_inf;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;
    Eigen::Vector3d eigen_curve_point;
    int differential_basis1 = 0;
    int differential_basis2 = 0;
    int basis_index = 0;
    double curvature_u_data = 0;
    double start_u = 0;
    double end_u = 0;
    int sub_intervals = 0;

    Curve_common CurveDesign;

    private_nh.getParam("/control_point", input_control_point);
    private_nh.getParam("/knot_vector", input_knot_vector);
    private_nh.getParam("/weight_vector", weight_vector);
    private_nh.param("visual1_differential_times", differential_basis1, 0);
    private_nh.param("visual2_differential_times", differential_basis2, 0);
    private_nh.param("basis_index", basis_index, 0);
    private_nh.param("curvature_u_data", curvature_u_data, 0.0);
    private_nh.param("Length_start_u", start_u, 0.0);
    private_nh.param("Length_end_u", end_u, 1.0);
    private_nh.param("Number_of_interval", sub_intervals, 20000);

    while(ros::ok())
    {
        points.header.frame_id = "odom";
        points.header.stamp = ros::Time::now();
        points.ns = "waypoints";
        points.action = visualization_msgs::Marker::ADD;
        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.scale.z = 0.1;
        points.color.r = 1; //red
        points.color.a = 1;

        //std::cout << "input control point size : " << input_control_point.size() << "\n";
        CurveDesign.ReadDiscreate2DPointFromLaunch(&control_point, input_control_point);
        CurveDesign.ShowDiscreatePoint(&points, control_point);
        CurveDesign.ReadSplineInf(&input_spline_inf, 3, control_point, input_knot_vector);
        
        //myCurve = CurveDesign.Generate_BsplineCurve(input_spline_inf, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBsplineCurve(input_spline_inf, 1, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis1, basis_index, t_intervel, frame_id);
        //myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis2, basis_index, t_intervel, frame_id);

        //test NURBS curve
        CurveDesign.ReadSplineInf(&input_spline_inf, weight_vector);
        myCurve = CurveDesign.Generate_NURBSCurve(input_spline_inf, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis1, basis_index, t_intervel, frame_id);
        //myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis2, basis_index, t_intervel, frame_id);
        //std::cout << "Curvature is : " << CurveDesign.CalculateCurvature(input_spline_inf, curvature_u_data, true) << "\n";
        std::cout << "Curvature radius is : " << CurveDesign.CalculateCurvatureRadius(input_spline_inf, curvature_u_data, true) << "\n";
        std::cout << "Curve total length is : " << CurveDesign.CalculateCurveLength(input_spline_inf, 0.0, 1.0, sub_intervals, true) << "\n";
        
        eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 0.25, true));
        std::cout << "Curve point in u = 0.25, x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";

        eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 0.5, true));
        std::cout << "Curve point in u = 0.5, x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";

        eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 0.75, true));
        std::cout << "Curve point in u = 0.75, x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";

        eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 1, true));
        std::cout << "Curve point in u = 1, x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";

        while(pub_discreate_maker.getNumSubscribers() == 0 && pub.getNumSubscribers() == 0 && pub_derivative_curve.getNumSubscribers() == 0)
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate_maker.publish(points);
        pub.publish(myCurve);
        pub_derivative_curve.publish(derivative_myCurve);
        ROS_INFO("end publish");

        ros::spin();        
    }
    
    return 0;
}