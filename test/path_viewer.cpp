#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "agv_path_smoothing/Curve_common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_viewer");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<nav_msgs::Path>("Curve_result", 1);

    nav_msgs::Path myCurve;
    geometry_msgs::Point start;
    geometry_msgs::Point goal;

    Curve_common CurveDesign;

    float t_intervel = 0.01;

    myCurve = CurveDesign.Generate_Line(start, goal, t_intervel);
    pub.publish(myCurve);

    ros::spin();
    return 0;
}