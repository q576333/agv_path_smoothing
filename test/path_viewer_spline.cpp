#include <ros/ros.h>

#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/conversion.h"

#include <geometry_msgs/PoseArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer_spline");
    ros::NodeHandle private_nh("~");

    double t_intervel = 0.01;

    ros::Publisher pub = private_nh.advertise<nav_msgs::Path>("Curve_result", 10, true);
    ros::Publisher pub_derivative_curve = private_nh.advertise<nav_msgs::Path>("Derivative_curve_result", 10, true);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10, true);
    ros::Publisher pub_curve_point_maker = private_nh.advertise<visualization_msgs::Marker>("Curve_point_maker", 10, true);

    nav_msgs::Path myCurve;
    nav_msgs::Path derivative_myCurve;
    visualization_msgs::Marker points;
    visualization_msgs::Marker curve_point;
    geometry_msgs::PoseArray curvature_pose;

    std::string frame_id = "odom";
    std::vector<double> input_control_point;
    std::vector<double> input_knot_vector;
    std::vector<double> weight_vector;
    Spline_Inf input_spline_inf;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;
    Eigen::Vector3d eigen_curve_point;
    Eigen::Vector3d eigen_curve_point_old;
    double curvepoint_angle;
    double curvepoint_angle_denom;

    Eigen::Vector3d curvature_vector;
    Eigen::Vector3d curvature_vector_old;
    double curvature_angle;
    double curvature_angle_denom;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > eigen_curve_point_vec;
    int differential_basis1 = 0;
    int differential_basis2 = 0;
    int basis_index = 0;
    double curvature_u_data = 0;
    double start_u = 0;
    double end_u = 0;
    int sub_intervals = 0;
    double curvature = 0;

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

        curve_point.header.frame_id = "odom";
        curve_point.header.stamp = ros::Time::now();
        curve_point.ns = "waypoints";
        curve_point.action = visualization_msgs::Marker::ADD;
        curve_point.id = 0;
        curve_point.type = visualization_msgs::Marker::SPHERE_LIST;
        curve_point.scale.x = 0.08;
        curve_point.scale.y = 0.08;
        curve_point.scale.z = 0.08;
        curve_point.color.b = 1; //blue
        curve_point.color.a = 1;

        //std::cout << "input control point size : " << input_control_point.size() << "\n";
        CurveDesign.ReadDiscreate2DPointFromLaunch(&control_point, input_control_point);
        CurveDesign.ShowDiscreatePoint(&points, control_point);
        CurveDesign.ReadSplineInf(&input_spline_inf, 3, control_point, input_knot_vector);
        
        //myCurve = CurveDesign.Generate_BsplineCurve(input_spline_inf, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBsplineCurve(input_spline_inf, 1, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis1, basis_index, t_intervel, frame_id);
        //myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis2, basis_index, t_intervel, frame_id);

        //test NURBS curve
        CurveDesign.ReadSplineInf(&input_spline_inf, weight_vector, false);
        myCurve = CurveDesign.Generate_NURBSCurve(input_spline_inf, t_intervel, frame_id);
        //derivative_myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis1, basis_index, t_intervel, frame_id);
        //myCurve = CurveDesign.Generate_DerivativeBasisFuncCurve(input_spline_inf, differential_basis2, basis_index, t_intervel, frame_id);
        //std::cout << "Curvature is : " << CurveDesign.CalculateCurvature(input_spline_inf, curvature_u_data, true) << "\n";
        std::cout << "Curvature radius is : " << CurveDesign.CalculateCurvatureRadius(input_spline_inf, curvature_u_data, true) << "\n";
        std::cout << "Curve total length is : " << CurveDesign.CalculateCurveLength(input_spline_inf, 0.0, 1.0, sub_intervals, true) << "\n";
        
        // eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 0.01, true));
        // std::cout << "Curve point in u = 0.01, x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";

        
        for(double u_test = 0; u_test <= 1; u_test += 0.02)
        {   
            std::cout << "-------------- u = "<< u_test << "--------------" << "\n";
            curvature_vector_old = curvature_vector;
            eigen_curve_point_old = eigen_curve_point;
            eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, u_test, true));
            std::cout << "Curve point x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";
            //std::cout << "Curvature is : " << CurveDesign.CalculateCurvature(input_spline_inf, u_test, true) << "\n";
            
            //Cacluate curve point vector angle
            curvepoint_angle_denom = eigen_curve_point_old.lpNorm<2>() * eigen_curve_point.lpNorm<2>();
            if(std::isnan(curvepoint_angle_denom))
            {
                std::cout << "this u no curvature angle " << "\n";
                curvepoint_angle = 0;
            }  
            else
                curvepoint_angle = std::acos(eigen_curve_point_old.dot(eigen_curve_point) / curvepoint_angle_denom);

            curvepoint_angle = curvepoint_angle * 180 / M_PI;
            std::cout << "Curve point angle is : " << curvepoint_angle << "\n";

            //Cacluate curvature vector angle
            curvature_vector = CurveDesign.CalculateCurvatureDirectionVector(input_spline_inf, u_test, true);
            curvature_angle_denom = curvature_vector_old.lpNorm<2>() * curvature_vector.lpNorm<2>();
            if(std::isnan(curvature_angle_denom))
            {
                std::cout << "this u no curvature angle " << "\n";
                curvature_angle = 0;
            }  
            else
                curvature_angle = std::acos(curvature_vector_old.dot(curvature_vector) / curvature_angle_denom);

            curvature_angle = curvature_angle * 180 / M_PI;
            std::cout << "Curvature angle is : " << curvature_angle << "\n";

            curvature = CurveDesign.CalculateCurvature(input_spline_inf, u_test, true);
            if(curvature_vector(0) < 0 && curvature_vector(1) < 0 || curvature_vector(0) > 0 && curvature_vector(1) < 0)
                 curvature *= -1;
            std::cout << "Curvature is : " << curvature << "\n";

            eigen_curve_point_vec.push_back(eigen_curve_point);
            std::cout << "\n";
        }

        eigen_curve_point = EigenVecter3dFromPointMsg(CurveDesign.CalculateCurvePoint(&input_spline_inf, 1, true));
        std::cout << "u = 1 Curve point x: " << eigen_curve_point(0) << " y: " << eigen_curve_point(1) << "\n";
        
        CurveDesign.ShowDiscreatePoint(&curve_point, eigen_curve_point_vec);
    
        while(pub_discreate_maker.getNumSubscribers() == 0 && pub.getNumSubscribers() == 0 && pub_derivative_curve.getNumSubscribers() == 0 && pub_curve_point_maker == 0)
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate_maker.publish(points);
        pub.publish(myCurve);
        pub_derivative_curve.publish(derivative_myCurve);
        pub_curve_point_maker.publish(curve_point);
        ROS_INFO("end publish");

        ros::spin();        
    }
    
    return 0;
}