#include <ros/ros.h>

#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/Curve_fitting.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "curve_fitting_viewer");
    ros::NodeHandle private_nh("~");

    double t_intervel = 0.01;

    ros::Publisher unlimit_curve_pub = private_nh.advertise<nav_msgs::Path>("unLimitCurveFitting_result", 1, true);
    ros::Publisher limit_curve_pub = private_nh.advertise<nav_msgs::Path>("limitCurveFitting_result", 1, true);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10, true);

    nav_msgs::Path unlimit_curve_path;
    nav_msgs::Path limit_curve_path;
    visualization_msgs::Marker points;
    visualization_msgs::Marker control_points;
    bool use_limit_derivative_;
    int input_order;
    double start_vector_weight_;
    double end_vector_weight_;
    std::string input_parameter_method = "Chord";
    std::string input_knotvector_method = "Average";
    std::string frame_id = "odom";
    
    Spline_Inf unlimit_curve_spline_inf;
    Spline_Inf limit_curve_spline_inf;
    Curve_common CurveDesign;
    Curve_fitting CurveFit;
    EigenTrajectoryPoint::Vector eigen_fitting_point;
    EigenTrajectoryPoint::Vector eigen_control_point;
    std::vector<double> input_fitting_point;
    std::vector<double> weight_vector;

    std::vector<double> derivative_vector_;
    geometry_msgs::Point derivative_point;
    std::vector<geometry_msgs::Point> derivative_points_vec;
    int derivative_point_index = 0;

    private_nh.getParam("/fitting_point", input_fitting_point);
    private_nh.getParam("/weight_vector", weight_vector);
    private_nh.getParam("/derivative_vector", derivative_vector_);
    private_nh.param("use_limit_derivative", use_limit_derivative_, false);
    private_nh.param("order", input_order, 3);
    private_nh.param("start_vector_weight", start_vector_weight_, 10.0);
    private_nh.param("end_vector_weight", end_vector_weight_, 10.0);
    private_nh.param("parameter_method", input_parameter_method, input_parameter_method);
    private_nh.param("knotvector_method", input_knotvector_method, input_knotvector_method);

    while(ros::ok())
    {
        points.header.frame_id = "odom";
        points.header.stamp = ros::Time::now();
        points.ns = "waypoints";
        points.action = visualization_msgs::Marker::ADD;
        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.scale.z = 0.2;
        points.color.b = 1; //red
        points.color.a = 1;

        control_points.header.frame_id = "odom";
        control_points.header.stamp = ros::Time::now();
        control_points.ns = "control_point";
        control_points.action = visualization_msgs::Marker::ADD;
        control_points.id = 0;
        control_points.type = visualization_msgs::Marker::SPHERE_LIST;
        control_points.scale.x = 0.1;
        control_points.scale.y = 0.1;
        control_points.scale.z = 0.1;
        control_points.color.r = 1; //blue
        control_points.color.a = 1;

        CurveDesign.ReadDiscreate2DPointFromLaunch(&eigen_fitting_point, input_fitting_point);
        CurveDesign.ShowDiscreatePoint(&points, eigen_fitting_point);

        if(use_limit_derivative_ == true)
        {
            for(int i = 0; i < 2; i++)
            {
                derivative_point.x = derivative_vector_.at(derivative_point_index);
                derivative_point.y = derivative_vector_.at(derivative_point_index + 1);
                derivative_points_vec.push_back(derivative_point);
                derivative_point_index += 2;
            }
            limit_curve_spline_inf = CurveFit.LimitCurveFitting(eigen_fitting_point, input_order, derivative_points_vec, dis_u_method::Chord, knotvector_method::LimitDerivative_Average, start_vector_weight_, end_vector_weight_);
            CurveDesign.ReadSplineInf(&limit_curve_spline_inf, weight_vector, use_limit_derivative_);
            limit_curve_path = CurveDesign.Generate_NURBSCurve(limit_curve_spline_inf, t_intervel, frame_id);
        }

        if(input_knotvector_method == "Average")
        {
            if(input_parameter_method == "Chord")
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Chord, knotvector_method::Average);
            else if(input_parameter_method == "Centripetal")
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Centripetal, knotvector_method::Average);
            else
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Average, knotvector_method::Average);
        }
        else
        {
            if(input_parameter_method == "Chord")
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Chord, knotvector_method::Equal_space);
            else if(input_parameter_method == "Centripetal")
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Centripetal, knotvector_method::Equal_space);
            else
                unlimit_curve_spline_inf = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Average, knotvector_method::Equal_space);
        }
            
        //unlimit_curve_path = CurveDesign.Generate_BsplineCurve(curve_fitting_result, t_intervel, frame_id);
        CurveDesign.ReadSplineInf(&unlimit_curve_spline_inf, weight_vector, false);
        unlimit_curve_path = CurveDesign.Generate_NURBSCurve(unlimit_curve_spline_inf, t_intervel, frame_id);
        
        CurveDesign.ShowDiscreatePoint(&control_points, limit_curve_spline_inf.control_point);
        
        if(use_limit_derivative_)
        {
            while(pub_discreate_maker.getNumSubscribers() == 0 && unlimit_curve_pub.getNumSubscribers() == 0 && limit_curve_pub.getNumSubscribers() == 0)
            {
                ROS_INFO("wait for subscriber");
                sleep(1);
            }
            limit_curve_pub.publish(limit_curve_path);
        }
        else
        {
            while(pub_discreate_maker.getNumSubscribers() == 0 && unlimit_curve_pub.getNumSubscribers() == 0)
            {
                ROS_INFO("wait for subscriber");
                sleep(1);
            }
        } 

        pub_discreate_maker.publish(points);
        pub_discreate_maker.publish(control_points);
        unlimit_curve_pub.publish(unlimit_curve_path);
        
        ROS_INFO("end publish");

        ros::spin();      
        ROS_INFO("after apin");  
    }
    
    return 0;
}