#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>

#include "agv_path_smoothing/Curve_common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer");
    ros::NodeHandle private_nh("~");

    double start_x = 0;
    double start_y = 0;
    double goal_x = 0;
    double goal_y = 0;
    double t_intervel = 0.001;
    Eigen::Vector2d a(-1, -1);
    Eigen::Vector2d b(5, 1);

    private_nh.param("start_point_x", start_x, -3.0);
    private_nh.param("start_point_y", start_y, -2.0);
    private_nh.param("goal_point_x", goal_x, 2.0);
    private_nh.param("goal_point_y", goal_y, 2.0);

    ros::Publisher pub = private_nh.advertise<nav_msgs::Path>("Curve_result", 10);
    ros::Publisher pub_discreate_maker = private_nh.advertise<visualization_msgs::Marker>("Discreate_point_maker", 10);

    nav_msgs::Path myCurve;
    geometry_msgs::Point start;
    geometry_msgs::Point goal;
    visualization_msgs::Marker points;

    Curve_common CurveDesign;

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

        start.x = start_x;
        start.y = start_y;
        points.points.push_back(start);

        goal.x = goal_x;
        goal.y = goal_y;
        points.points.push_back(goal);

        myCurve = CurveDesign.Generate_Line(start, goal, t_intervel);

        while(pub_discreate_maker.getNumSubscribers() == 0 && pub.getNumSubscribers() == 0)
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate_maker.publish(points);
        pub.publish(myCurve);
        ROS_INFO("end publish");

        std::cout << "Eigen add result : " << a + b << "\n";

        ros::spin();        
    }
    
    return 0;
}