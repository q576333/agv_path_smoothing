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

    std::cout << "End cacluation" << "\n";
    return line_result;
}

nav_msgs::Path Curve_common::Generate_BezierCurve(EigenTrajectoryPoint::Vector control_point, double t_intervel)
{
    nav_msgs::Path bezier_curve_result;
    geometry_msgs::PoseStamped current_pose;
    EigenTrajectoryPoint::Vector temp_control_point_vec;
    
    bezier_curve_result.header.frame_id = "odom";
    bezier_curve_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "odom";

    int segment = 1 / t_intervel;
    int control_point_length = control_point.size();
    double curve_parameter = 0;
    temp_control_point_vec.reserve(control_point_length);

    for(int i = 0; i <= segment; i++)
    {
        temp_control_point_vec.assign(control_point.begin(), control_point.end());
        
        for(int j = 1; j <= control_point_length - 1; j++)
        {            
            for(int k = 1; k <= control_point_length - j; k++)
            {
                temp_control_point_vec.at(k - 1).position(0) = temp_control_point_vec.at(k - 1).position(0) * (1 - curve_parameter) + temp_control_point_vec.at(k).position(0) * curve_parameter;
                temp_control_point_vec.at(k - 1).position(1) = temp_control_point_vec.at(k - 1).position(1) * (1 - curve_parameter) + temp_control_point_vec.at(k).position(1) * curve_parameter;            
            }
        }
        current_pose.header.seq = i;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = temp_control_point_vec.at(0).position(0);
        current_pose.pose.position.y = temp_control_point_vec.at(0).position(1);
        bezier_curve_result.poses.push_back(current_pose);
        curve_parameter += t_intervel;
    }

    std::cout << "End cacluation" << "\n";
    return bezier_curve_result;
}

void Curve_common::ReadControlPointFromLaunch(EigenTrajectoryPoint::Vector *input_control_point, std::vector<double> input_point)
{
    int index = 0;
    Eigen::Vector3d read_point;
    EigenTrajectoryPoint dis_control_point;
    input_control_point->reserve(input_point.size() / 2);

    for(int i = 0; i < input_point.size(); i++)
    {
        if(i % 2 == 0)
        {
            read_point(0) = input_point[i];
        }
        else
        {
            read_point(1) = input_point[i];
            dis_control_point.position = read_point;
            //input_control_point->emplace_back(dis_control_point); //I don't know why have error
            input_control_point->push_back(dis_control_point);         
            index++;
        }        
    }

    //Debug use
    // std::cout << "input control point size : " << input_control_point->size() << "\n";
    // for(int i = 0; i < input_control_point->size(); i++)
    // {
    //     std::cout << "control point x : " << input_control_point->at(i).position(0) << "\n";
    //     std::cout << "control point y : " << input_control_point->at(i).position(1) << "\n";
    // }
}

void Curve_common::ShowControlPoint(visualization_msgs::Marker *points, EigenTrajectoryPoint::Vector input_control_point)
{
    geometry_msgs::Point view_point;
    for(int i = 0; i < input_control_point.size(); i++)
    {
        view_point.x = input_control_point.at(i).position(0);
        view_point.y = input_control_point.at(i).position(1);
        points->points.push_back(view_point);
    }
}