#include <ros/ros.h>

#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/Curve_fitting.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "curve_fitting_viewer");
    ros::NodeHandle private_nh("~");

    double t_intervel = 0.01;

    ros::Publisher pub = private_nh.advertise<nav_msgs::Path>("Curve_fitting_result", 1000);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10);

    nav_msgs::Path myCurve;
    visualization_msgs::Marker points;
    visualization_msgs::Marker control_points;
    int input_order;
    std::string input_parameter_method = "Chord";
    std::string input_knotvector_method = "Average";
    std::string frame_id = "odom";
    
    Spline_Inf curve_fitting_result;
    Curve_common CurveDesign;
    Curve_fitting CurveFit;
    EigenTrajectoryPoint::Vector eigen_fitting_point;
    std::vector<double> input_fitting_point;
    EigenTrajectoryPoint::Vector eigen_control_point;

    private_nh.getParam("/fitting_point", input_fitting_point);
    private_nh.param("order", input_order, 3);
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
        points.color.r = 1; //red
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
        control_points.color.b = 1; //blue
        control_points.color.a = 1;

        CurveDesign.ReadDiscreate2DPointFromLaunch(&eigen_fitting_point, input_fitting_point);
        CurveDesign.ShowDiscreatePoint(&points, eigen_fitting_point);

        if(input_knotvector_method == "Average")
        {
            if(input_parameter_method == "Chord")
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Chord, knotvector_method::Average);
            else if(input_parameter_method == "Centripetal")
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Centripetal, knotvector_method::Average);
            else
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Average, knotvector_method::Average);
        }
        else
        {
            if(input_parameter_method == "Chord")
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Chord, knotvector_method::Equal_space);
            else if(input_parameter_method == "Centripetal")
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Centripetal, knotvector_method::Equal_space);
            else
                curve_fitting_result = CurveFit.UnLimitCurveFitting(eigen_fitting_point, input_order, dis_u_method::Average, knotvector_method::Equal_space);
        }
            
        myCurve = CurveDesign.Generate_BsplineCurve(curve_fitting_result, t_intervel, frame_id);
        
        CurveDesign.ShowDiscreatePoint(&control_points, curve_fitting_result.control_point);
        
        while(pub_discreate_maker.getNumSubscribers() == 0 && pub.getNumSubscribers() == 0)
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate_maker.publish(points);
        pub_discreate_maker.publish(control_points);
        pub.publish(myCurve);
        ROS_INFO("end publish");

        ros::spin();      
        ROS_INFO("after apin");  
    }
    
    return 0;
}