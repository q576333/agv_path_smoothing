#include <ros/ros.h>

#include "agv_path_smoothing/Curve_common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer_spline");
    ros::NodeHandle private_nh("~");

    double t_intervel = 0.01;

    ros::Publisher pub = private_nh.advertise<nav_msgs::Path>("Curve_result", 10);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10);

    nav_msgs::Path myCurve;
    visualization_msgs::Marker points;
    std::vector<double> input_control_point;
    std::vector<double> input_knot_vector;
    Spline_Inf input_spline_inf;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;

    Curve_common CurveDesign;

    private_nh.getParam("/control_point", input_control_point);
    private_nh.getParam("/knot_vector", input_knot_vector);

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
        
        myCurve = CurveDesign.Generate_BsplineCurve(input_spline_inf, t_intervel);

        while(pub_discreate_maker.getNumSubscribers() == 0 && pub.getNumSubscribers() == 0)
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate_maker.publish(points);
        pub.publish(myCurve);
        ROS_INFO("end publish");

        ros::spin();        
    }
    
    return 0;
}