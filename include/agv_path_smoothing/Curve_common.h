#ifndef AGV_PATH_SMOOTHING_CURVE_COMMON_H_
#define AGV_PATH_SMOOTHING_CURVE_COMMON_H_

#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"

class Curve_common
{
    public:
        Curve_common();
        nav_msgs::Path Generate_Line(geometry_msgs::Point start_point, geometry_msgs::Point end_point, double t_intervel);
    private:
};
#endif //AGV_PATH_SMOOTHING_CURVE_COMMON_H_