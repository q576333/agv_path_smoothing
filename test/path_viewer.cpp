#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include "agv_path_smoothing/Curve_common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double start_x = 0;
    double start_y = 0;
    double goal_x = 0;
    double goal_y = 0;
    double t_intervel = 0.001;

    private_nh.param("start_point_x", start_x, -3.0);
    private_nh.param("start_point_y", start_y, -2.0);
    private_nh.param("goal_point_x", goal_x, 2.0);
    private_nh.param("goal_point_y", goal_y, 2.0);

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("Curve_result", 100);
    ros::Publisher pub_discreate = nh.advertise<nav_msgs::Path>("Discreate_point", 100);

    nav_msgs::Path myCurve;
    //geometry_msgs::PoseStamped input_point;
    nav_msgs::Path visual_input;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

    Curve_common CurveDesign;

    while(ros::ok())
    {
        visual_input.header.seq = 0;
        visual_input.header.frame_id = "odom";
    
        start.header.frame_id = "odom";
        start.header.seq = 0;
        start.header.stamp = ros::Time::now();
        start.pose.position.x = start_x;
        start.pose.position.y = start_y;
        visual_input.poses.push_back(start);

        goal.header.frame_id = "odom";
        goal.header.seq = 1;
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_x;
        visual_input.poses.push_back(goal);

        //myCurve = CurveDesign.Generate_Line(start, goal, t_intervel);
        //&& pub.getNumSubscribers() == 0

        while(pub_discreate.getNumSubscribers() == 0 )
        {
            ROS_INFO("wait for subscriber");
            sleep(1);
        }
        
        pub_discreate.publish(visual_input);
        //pub.publish(myCurve);
        ROS_INFO("end publish");
        //pub_talker.publish(data);

        ros::spin();        
    }
    
    return 0;
}