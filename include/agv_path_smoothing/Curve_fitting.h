#ifndef AGV_PATH_SMOOTHING_CURVE_FITTING_H_
#define AGV_PATH_SMOOTHING_CURVE_FITTING_H_

#include "agv_path_smoothing/Curve_common.h"
#include <Eigen/Eigen>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

typedef enum class DiscreateParameterMethod
{
    Average,
    Chord,
    Centripetal
}dis_u_method;

typedef enum class KnotVectorMethod
{
    Equal_space,
    Average,
    LimitDerivative_Average //for LimitCurveFitting use
}knotvector_method;

class Curve_fitting
{
    public:
        Curve_fitting();
        
        Spline_Inf UnLimitCurveFitting(EigenTrajectoryPoint::Vector input_point, int order, dis_u_method parameter_method, knotvector_method knot_method);
        Spline_Inf LimitCurveFitting(EigenTrajectoryPoint::Vector input_point, int order, std::vector<geometry_msgs::Point> derivatives_spec, dis_u_method parameter_method, knotvector_method knot_method, double start_vector_weight, double end_vector_weight);
    private:
        void CalculateDiscreatePointParameter(EigenTrajectoryPoint::Vector *input_point, dis_u_method parameter_method);
        void CalculateKnotVector(EigenTrajectoryPoint::Vector input_point, Spline_Inf *curvefit_inf, knotvector_method knot_method);
};
#endif //AGV_PATH_SMOOTHING_CURVE_FITTING_H_