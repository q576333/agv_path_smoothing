#ifndef AGV_PATH_SMOOTHING_CURVE_COMMON_H_
#define AGV_PATH_SMOOTHING_CURVE_COMMON_H_

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// struct Point
// {
//     double x;
//     double y;
// };

class Curve_common
{
    public:
        Curve_common();
        nav_msgs::Path Generate_Line(geometry_msgs::PoseStamped start_point, geometry_msgs::PoseStamped end_point, double t_intervel);
    private:
};
#endif //AGV_PATH_SMOOTHING_CURVE_COMMON_H_