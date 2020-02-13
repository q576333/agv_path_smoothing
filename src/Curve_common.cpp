#include "agv_path_smoothing/Curve_common.h"

#include <geometry_msgs/PoseStamped.h>

#include <iostream>

Curve_common::Curve_common()
{

}

nav_msgs::Path Curve_common::Generate_Line(geometry_msgs::Point start_point, geometry_msgs::Point end_point, double t_intervel)
{
    nav_msgs::Path line_result;
    geometry_msgs::PoseStamped current_pose;
    
    line_result.header.frame_id = "odom";
    line_result.header.stamp = ros::Time::now();

    current_pose.header.frame_id = "odom";

    int segment = 1 / t_intervel;
    double line_parameter = 0;

    for(int i = 0; i < segment; i++)
    {
        current_pose.header.seq = i;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = start_point.x * (1 - line_parameter) + end_point.x * (line_parameter);
        current_pose.pose.position.y = start_point.y * (1 - line_parameter) + end_point.y * (line_parameter);

        line_result.poses.push_back(current_pose);
        line_parameter += t_intervel;
    }

    std::cout << "end cacluation" << "\n";
    return line_result;
}