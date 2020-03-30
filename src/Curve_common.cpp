#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/conversion.h"

#include <geometry_msgs/PoseStamped.h>

#include <iostream>

Curve_common::Curve_common()
{

}

nav_msgs::Path Curve_common::Generate_Line(geometry_msgs::Point start_point, geometry_msgs::Point end_point, double t_intervel, std::string frame_id)
{
    nav_msgs::Path line_result;
    geometry_msgs::PoseStamped current_pose;
    
    line_result.header.frame_id = frame_id;
    line_result.header.stamp = ros::Time::now();

    current_pose.header.frame_id = frame_id;

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

nav_msgs::Path Curve_common::Generate_BezierCurve(EigenTrajectoryPoint::Vector control_point, double t_intervel, std::string frame_id)
{
    nav_msgs::Path bezier_curve_result;
    geometry_msgs::PoseStamped current_pose;
    EigenTrajectoryPoint::Vector temp_control_point_vec;
    
    bezier_curve_result.header.frame_id = frame_id;
    bezier_curve_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = frame_id;

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

    std::cout << "End calculate Bezier Curve" << "\n";
    return bezier_curve_result;
}

void Curve_common::ReadDiscreate2DPointFromLaunch(EigenTrajectoryPoint::Vector *input_point, std::vector<double> file_discreate_point)
{
    int index = 0;
    Eigen::Vector3d read_point;
    EigenTrajectoryPoint eigen_discreate_point;
    input_point->reserve(file_discreate_point.size() / 2);

    for(int i = 0; i < file_discreate_point.size(); i++)
    {
        if(i % 2 == 0)
        {
            read_point(0) = file_discreate_point[i];
        }
        else
        {
            read_point(1) = file_discreate_point[i];
            eigen_discreate_point.position = read_point;
            //input_point->emplace_back(dis_control_point); //I don't know why have error
            input_point->push_back(eigen_discreate_point);         
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

void Curve_common::ReadDiscreate2DPointFromLaunch(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *input_point, std::vector<double> file_discreate_point)
{
    Eigen::Vector3d read_point;
    input_point->reserve(file_discreate_point.size() / 2);

    for(int i = 0; i < file_discreate_point.size(); i++)
    {
        if(i % 2 == 0)
        {
            read_point(0) = file_discreate_point[i];
        }
        else
        {
            read_point(1) = file_discreate_point[i];
            input_point->push_back(read_point);         
        }        
    }

    //Debug use
    // std::cout << "input control point size : " << input_point->size() << "\n";
    // for(int i = 0; i < input_point->size(); i++)
    // {
    //     std::cout << "control point x : " << input_point->at(i)(0) << "\n";
    //     std::cout << "control point y : " << input_point->at(i)(1) << "\n";
    // }
}

visualization_msgs::Marker Curve_common::ShowDiscreatePoint(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point, const std::string& frame_id, const std::string& name, double scale) 
{
    visualization_msgs::Marker waypoints_marker;

    waypoints_marker.header.frame_id = frame_id;
    waypoints_marker.header.stamp = ros::Time::now();
    waypoints_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //waypoints_marker.color = color;
    waypoints_marker.color.r = 1;
    waypoints_marker.color.g = 1;
    waypoints_marker.color.a = 0.75;
    waypoints_marker.ns = name;
    waypoints_marker.scale.x = scale;
    waypoints_marker.scale.y = scale;
    waypoints_marker.scale.z = scale;

    waypoints_marker.points.reserve(discreate_point.size());

    geometry_msgs::Point view_point;
    for(int i = 0; i < discreate_point.size(); i++)
    {
        view_point.x = discreate_point.at(i)(0);
        view_point.y = discreate_point.at(i)(1);
        waypoints_marker.points.push_back(view_point);
    }

    return waypoints_marker;
}

visualization_msgs::Marker Curve_common::ShowDiscreatePoint(EigenTrajectoryPoint::Vector& discreate_point, const std::string& frame_id, const std::string& name, double scale) 
{
    visualization_msgs::Marker waypoints_marker;

    waypoints_marker.header.frame_id = frame_id;
    waypoints_marker.header.stamp = ros::Time::now();
    waypoints_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //waypoints_marker.color = color;
    waypoints_marker.color.r = 1;
    waypoints_marker.color.g = 1;
    waypoints_marker.color.a = 0.75;
    waypoints_marker.ns = name;
    waypoints_marker.scale.x = scale;
    waypoints_marker.scale.y = scale;
    waypoints_marker.scale.z = scale;

    waypoints_marker.points.reserve(discreate_point.size());

    geometry_msgs::Point view_point;
    for(int i = 0; i < discreate_point.size(); i++)
    {
        view_point.x = discreate_point.at(i).position(0);
        view_point.y = discreate_point.at(i).position(1);
        waypoints_marker.points.push_back(view_point);
    }

    return waypoints_marker;
}

void Curve_common::ShowDiscreatePoint(visualization_msgs::Marker *points, EigenTrajectoryPoint::Vector discreate_point)
{
    geometry_msgs::Point view_point;
    for(int i = 0; i < discreate_point.size(); i++)
    {
        view_point.x = discreate_point.at(i).position(0);
        view_point.y = discreate_point.at(i).position(1);
        points->points.push_back(view_point);
    }
}

void Curve_common::ShowDiscreatePoint(visualization_msgs::Marker *points, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > discreate_point)
{
    geometry_msgs::Point view_point;
    for(int i = 0; i < discreate_point.size(); i++)
    {
        view_point.x = discreate_point.at(i)(0);
        view_point.y = discreate_point.at(i)(1);
        points->points.push_back(view_point);
    }
}

void Curve_common::ReadSplineInf(Spline_Inf *bspline_inf, int order, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point, std::vector<double> knot_vector)
{
    bspline_inf->order = order;
    bspline_inf->knot_vector.assign(knot_vector.begin(), knot_vector.end());
    bspline_inf->control_point.assign(control_point.begin(), control_point.end());

    //Debug use
    // std::cout << "knot vector size : " << bspline_inf->knot_vector.size() << "\n";
    // for(int i = 0; i < bspline_inf->knot_vector.size(); i++)
    // {
    //     std::cout << "knot vector x : " << bspline_inf->knot_vector.at(i) << "\n";    
    // }

    // std::cout << "input control point size : " << bspline_inf->control_point.size() << "\n";
    // for(int i = 0; i < bspline_inf->control_point.size(); i++)
    // {
    //     std::cout << "control point x : " << bspline_inf->control_point.at(i)(0) << ", y = " << bspline_inf->control_point.at(i)(1) << "\n";
    // }
}

void Curve_common::ReadSplineInf(Spline_Inf *spline_inf, std::vector<double> weight_vector)
{
    if(spline_inf->control_point.size() != weight_vector.size())
    {
        std::cout << "weight vector size is wrong" << "\n";
        return;
    }
    spline_inf->weight.assign(weight_vector.begin(), weight_vector.end());
}

nav_msgs::Path Curve_common::Generate_BsplineCurve(Spline_Inf bspline_inf, double t_intervel, std::string frame_id)
{
    nav_msgs::Path bspline_curve_result;
    geometry_msgs::PoseStamped current_pose;
    
    bspline_curve_result.header.frame_id = frame_id;
    bspline_curve_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = frame_id;

    int p_degree = bspline_inf.order - 1;
    int n = bspline_inf.control_point.size() - 1;
    //TODO: Check knot vector size and sequence is correect
    int m = bspline_inf.knot_vector.size() - 1; //The last knot vector index number
    int segment = 1 / t_intervel;
    double delta_t = 100;
    double curve_parameter = 0;
    double left_denom = 0, right_denom = 0;
    double left_term = 0, right_term = 0;
    double sum_x = 0, sum_y = 0;
    std::vector<double> curve_parameter_vec;
    Eigen::VectorXd temp_basic_function;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > temp_basic_function_eigen;

    //Debug use
    // for(int i = 0; i < bspline_inf.knot_vector.size(); i++)
    // {
    //     std::cout << "knot vector x : " << bspline_inf.knot_vector.at(i) << "\n";    
    // }

    // std::cout << "input control point size : " << bspline_inf.control_point.size() << "\n";
    // for(int i = 0; i < bspline_inf.control_point.size(); i++)
    // {
    //     std::cout << "control point x : " << bspline_inf.control_point.at(i)(0) << ", y = " << bspline_inf.control_point.at(i)(1) << "\n";
    // }

    //TODO: Can optimal this part code
    delta_t = 1 / (double)(segment - 1);
    curve_parameter_vec.resize(segment);
    for(int u = 0; u < segment; u++)
    {
        curve_parameter_vec[u] = curve_parameter;
        curve_parameter += delta_t;
    }

    bspline_inf.N.clear();
    temp_basic_function.resize(segment);

    //TODO: Check basic function boundary is correct
    //Calculate degree = 0's basic function
    for(int i = 0; i < m; i++)
    {        
        if(bspline_inf.knot_vector.at(i) != bspline_inf.knot_vector.at(n))
        {
            for(int u = 0; u < segment; u++)
            {
                if(bspline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] < bspline_inf.knot_vector.at(i+1))                        
                    temp_basic_function(u) = 1;
                else
                    temp_basic_function(u) = 0;

                //std::cout << "small temp basic function index : " << u << " small temp basic function value : " << temp_basic_function(u) << "\n";
                //std::cout << "small temp basic function value : " << temp_basic_function(u) << "\n";
            }
        }
        else
        {
            for(int u = 0; u < segment; u++)
            {
                if(bspline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] <= bspline_inf.knot_vector.at(i+1))                        
                    temp_basic_function(u) = 1;
                else
                    temp_basic_function(u) = 0;

                //std::cout << "temp basic function index : " << u << "temp basic function value : " << temp_basic_function(u) << "\n";
                //std::cout << "small temp basic function value : " << temp_basic_function(u) << "\n";
            }
        }
        

        // if(bspline_inf.knot_vector.at(i) < bspline_inf.knot_vector.at(i+1))
        // {
        //     if(bspline_inf.knot_vector.at(i) != bspline_inf.knot_vector.at(n))
        //     {
        //         for(int u = 0; u < segment; u++)
        //         {
        //             if(bspline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] < bspline_inf.knot_vector.at(i+1))                        
        //                 temp_basic_function(u) = 1;
        //             else
        //                 temp_basic_function(u) = 0;

        //             std::cout << "small temp basic function index : " << u << "   small temp basic function value : " << temp_basic_function(u) << "\n";
        //         }
        //     }
        //     else
        //     {
        //         for(int u = 0; u < segment; u++)
        //         {
        //             if(bspline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] <= bspline_inf.knot_vector.at(i+1))                        
        //                 temp_basic_function(u) = 1;
        //             else
        //                 temp_basic_function(u) = 0;

        //             std::cout << "n basic function index : " << u << "  n basic function value : " << temp_basic_function(u) << "\n";
        //             //std::cout << "small temp basic function value : " << temp_basic_function(u) << "\n";
        //         }
        //     }
        // }
        // else
        // {
        //     for(int u = 0; u < segment; u++)
        //     {
        //         if(bspline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] <= bspline_inf.knot_vector.at(i+1))                        
        //             temp_basic_function(u) = 1;
        //         else
        //             temp_basic_function(u) = 0;

        //         std::cout << "temp basic function index : " << u << "   temp basic function value : " << temp_basic_function(u) << "\n";
        //     }
        // }
        bspline_inf.N.push_back(temp_basic_function);   
        //std::cout << i << "'s bspline_inf Ni,0 size " << temp_basic_function.size() << "\n";
    }

    //Calculate the rest of basic function
    for(int p = 1; p <= p_degree; p++)
    {
        temp_basic_function_eigen.clear();
        for(int i = 0; i < (m - p); i++)
        {
            for(int u = 0; u < segment; u++)
            {
                left_denom = bspline_inf.knot_vector.at(p+i) - bspline_inf.knot_vector.at(i);
                right_denom = bspline_inf.knot_vector.at(i+p+1) - bspline_inf.knot_vector.at(i+1);

                if(left_denom == 0)
                    left_term = 0;
                else
                    left_term = (curve_parameter_vec[u] - bspline_inf.knot_vector.at(i)) * bspline_inf.N.at(i)(u) / left_denom;

                if(right_denom == 0)
                    right_term = 0;
                else
                    right_term = (bspline_inf.knot_vector.at(i+p+1) - curve_parameter_vec[u]) * bspline_inf.N.at(i+1)(u) / right_denom;
                
                temp_basic_function(u) = left_term + right_term;            
            }
            temp_basic_function_eigen.push_back(temp_basic_function);
        }
        bspline_inf.N = temp_basic_function_eigen;
    }

    for(int u = 0; u < segment; u++)
    {   
        sum_x = 0;
        sum_y = 0;
        for(int i = 0; i < bspline_inf.N.size(); i++)
        { 
            sum_x += bspline_inf.control_point.at(i)(0) * bspline_inf.N.at(i)(u);
            sum_y += bspline_inf.control_point.at(i)(1) * bspline_inf.N.at(i)(u);  
            //std::cout << i << "'s ," << u <<"'s bspline_inf value : " << bspline_inf.N.at(i)(u) << "\n";
        }
        // std::cout << u << "'s current_pose x : " << sum_x << "\n";
        // std::cout << u << "'s current_pose y : " << sum_y << "\n";
        current_pose.header.seq = u;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = sum_x;
        current_pose.pose.position.y = sum_y;
        bspline_curve_result.poses.push_back(current_pose);        
    }

    std::cout << "End calculate bspline function" << "\n";
    return bspline_curve_result;
}

nav_msgs::Path Curve_common::Generate_NURBSCurve(Spline_Inf spline_inf, double t_intervel, std::string frame_id)
{
    nav_msgs::Path nurbs_curve_result;
    geometry_msgs::PoseStamped current_pose;
    
    nurbs_curve_result.header.frame_id = frame_id;
    nurbs_curve_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = frame_id;

    int p_degree = spline_inf.order - 1;
    int n = spline_inf.control_point.size() - 1;
    //TODO: Check knot vector size and sequence is correect
    int m = spline_inf.knot_vector.size() - 1; //The last knot vector index number
    int segment = 1 / t_intervel;
    double delta_t = 100;
    double curve_parameter = 0;
    double left_denom = 0, right_denom = 0;
    double left_term = 0, right_term = 0;
    double sum_nurbs_denom = 0, sum_nurbs_fract = 0;
    double sum_x = 0, sum_y = 0;
    std::vector<double> curve_parameter_vec;
    Eigen::VectorXd temp_basic_function;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > temp_basic_function_eigen;

    //Debug use
    // for(int i = 0; i < bspline_inf.knot_vector.size(); i++)
    // {
    //     std::cout << "knot vector x : " << bspline_inf.knot_vector.at(i) << "\n";    
    // }

    // std::cout << "input control point size : " << bspline_inf.control_point.size() << "\n";
    // for(int i = 0; i < bspline_inf.control_point.size(); i++)
    // {
    //     std::cout << "control point x : " << bspline_inf.control_point.at(i)(0) << ", y = " << bspline_inf.control_point.at(i)(1) << "\n";
    // }

    if(spline_inf.weight.size() == 0)
    {
        std::cout << "weight vector size is zero, please call read ReadSplineInf function" << "\n";
        return nurbs_curve_result;
    }

    //TODO: Can optimal this part code
    delta_t = 1 / (double)(segment - 1);
    curve_parameter_vec.resize(segment);
    for(int u = 0; u < segment; u++)
    {
        curve_parameter_vec[u] = curve_parameter;
        curve_parameter += delta_t;
    }

    spline_inf.N.clear();
    temp_basic_function.resize(segment);

    //TODO: Check basic function boundary is correct
    //Calculate degree = 0's basic function
    for(int i = 0; i < m; i++)
    {        
        //if(spline_inf.knot_vector.at(i) != spline_inf.knot_vector.at(n))
        if(spline_inf.knot_vector.at(i) != spline_inf.knot_vector.at(i+1))
        {
            for(int u = 0; u < segment; u++)
            {
                if(spline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] < spline_inf.knot_vector.at(i+1))                        
                    temp_basic_function(u) = 1;
                else
                    temp_basic_function(u) = 0;

                //std::cout << "small temp basic function index : " << u << " small temp basic function value : " << temp_basic_function(u) << "\n";
                //std::cout << "small temp basic function value : " << temp_basic_function(u) << "\n";
            }
        }
        else
        {
            for(int u = 0; u < segment; u++)
            {
                if(spline_inf.knot_vector.at(i) <= curve_parameter_vec[u] && curve_parameter_vec[u] <= spline_inf.knot_vector.at(i+1))                        
                    temp_basic_function(u) = 1;
                else
                    temp_basic_function(u) = 0;

                //std::cout << "temp basic function index : " << u << "temp basic function value : " << temp_basic_function(u) << "\n";
                //std::cout << "small temp basic function value : " << temp_basic_function(u) << "\n";
            }
        }
        
        spline_inf.N.push_back(temp_basic_function);   
        //std::cout << i << "'s bspline_inf Ni,0 size " << temp_basic_function.size() << "\n";
    }

    //Calculate the rest of basic function
    for(int p = 1; p <= p_degree; p++)
    {
        temp_basic_function_eigen.clear();
        for(int i = 0; i < (m - p); i++) //m-p is calculate how many region between basis function
        {
            for(int u = 0; u < segment; u++)
            {
                left_denom = spline_inf.knot_vector.at(p+i) - spline_inf.knot_vector.at(i);
                right_denom = spline_inf.knot_vector.at(i+p+1) - spline_inf.knot_vector.at(i+1);

                if(left_denom == 0)
                    left_term = 0;
                else
                    left_term = (curve_parameter_vec[u] - spline_inf.knot_vector.at(i)) * spline_inf.N.at(i)(u) / left_denom;

                if(right_denom == 0)
                    right_term = 0;
                else
                    right_term = (spline_inf.knot_vector.at(i+p+1) - curve_parameter_vec[u]) * spline_inf.N.at(i+1)(u) / right_denom;
                
                temp_basic_function(u) = left_term + right_term;            
            }
            temp_basic_function_eigen.push_back(temp_basic_function);
        }
        spline_inf.N = temp_basic_function_eigen;
    }

    for(int u = 0; u < segment; u++)
    {   
        sum_nurbs_denom = 0;
        sum_x = 0;
        sum_y = 0;
        for(int i = 0; i < spline_inf.N.size(); i++)
        { 
            sum_nurbs_denom += spline_inf.N.at(i)(u) * spline_inf.weight.at(i);
            sum_x += spline_inf.control_point.at(i)(0) * spline_inf.N.at(i)(u) * spline_inf.weight.at(i);
            sum_y += spline_inf.control_point.at(i)(1) * spline_inf.N.at(i)(u) * spline_inf.weight.at(i);  
            //std::cout << i << "'s ," << u <<"'s bspline_inf value : " << bspline_inf.N.at(i)(u) << "\n";
        }
        sum_x /= sum_nurbs_denom;
        sum_y /= sum_nurbs_denom;
        // std::cout << u << "'s current_pose x : " << sum_x << "\n";
        // std::cout << u << "'s current_pose y : " << sum_y << "\n";
        current_pose.header.seq = u;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = sum_x;
        current_pose.pose.position.y = sum_y;
        nurbs_curve_result.poses.push_back(current_pose);        
    }

    std::cout << "End calculate NURBS function" << "\n";
    return nurbs_curve_result;
}

void Curve_common::CalculateDerivativeBasisFunc(Spline_Inf *spline_inf, double u_data, int differential_times)
{
    int p_degree = spline_inf->order - 1;
    int n = spline_inf->control_point.size() - 1;
    //TODO: Check knot vector size and sequence is correect
    int m = spline_inf->knot_vector.size() - 1; //The last knot vector index number
    double left_denom = 0, right_denom = 0;
    double left_term = 0, right_term = 0;
    double derivative_left_term = 0, derivative_right_term = 0;
    double derivative_value = 0;
    std::vector<double> temp_basic_function;
    Eigen::VectorXd derivative_temp_basic_function;
    double degree_basis_value;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > derivative_basic_function_eigen;

    //Calculate degree = 0's basic function
    spline_inf->N_double_vec.clear();
    spline_inf->N_double_vec.resize(m);
    for(int i = 0; i < m; i++)
    {
        // if(spline_inf->knot_vector.at(i) != spline_inf->knot_vector.at(n))
        if(spline_inf->knot_vector.at(i) != spline_inf->knot_vector.at(i+1))
        {
            if(spline_inf->knot_vector.at(i) <= u_data && u_data < spline_inf->knot_vector.at(i+1))                        
                degree_basis_value = 1;
            else
                degree_basis_value = 0;
        }
        else
        { 
            if(spline_inf->knot_vector.at(i) <= u_data && u_data <= spline_inf->knot_vector.at(i+1))                        
                degree_basis_value = 1;
            else
                degree_basis_value = 0;
        }

        spline_inf->N_double_vec.at(i) = degree_basis_value;
    }

    //Calculate the rest of basic function
    spline_inf->dN.clear();
    for(int p = 1; p <= p_degree; p++)
    {        
        temp_basic_function.clear();
        derivative_temp_basic_function.resize(m-p);
        for(int i = 0; i < (m - p); i++)
        {
            left_denom = spline_inf->knot_vector.at(p+i) - spline_inf->knot_vector.at(i);
            right_denom = spline_inf->knot_vector.at(i+p+1) - spline_inf->knot_vector.at(i+1);

            if(left_denom == 0)
            {
                left_term = 0;
                derivative_left_term = 0;
            }    
            else
            {
                left_term = (u_data - spline_inf->knot_vector.at(i)) * spline_inf->N_double_vec.at(i) / left_denom;
                derivative_left_term = spline_inf->N_double_vec.at(i) / left_denom;
            }
                
            if(right_denom == 0)
            {
                right_term = 0;
                derivative_right_term = 0;
            }                
            else
            {
                right_term = (spline_inf->knot_vector.at(i+p+1) - u_data) * spline_inf->N_double_vec.at(i+1) / right_denom;
                derivative_right_term = spline_inf->N_double_vec.at(i+1) / right_denom;
            }
                               
            derivative_value = p * (derivative_left_term - derivative_right_term);              
            temp_basic_function.push_back(left_term + right_term);  //Calculate Ni,p
            derivative_temp_basic_function(i) = derivative_value;  //Calculate dNi,p
        }
        spline_inf->N_double_vec.assign(temp_basic_function.begin(), temp_basic_function.end());
        spline_inf->dN.push_back(derivative_temp_basic_function); 
    }
    
    if(differential_times > 1)
    {
        spline_inf->ddN.clear();
        for(int p = 2; p <= p_degree; p++)
        {
            derivative_temp_basic_function.resize(m-p);
            for(int i = 0; i < (m - p); i++)
            {
                left_denom = spline_inf->knot_vector.at(p+i) - spline_inf->knot_vector.at(i);
                right_denom = spline_inf->knot_vector.at(i+p+1) - spline_inf->knot_vector.at(i+1);

                if(left_denom == 0)            
                    derivative_left_term = 0;  
                else
                    derivative_left_term = spline_inf->dN.at(p-2)(i) / left_denom;
                    
                if(right_denom == 0)
                    derivative_right_term = 0;               
                else
                    derivative_right_term = spline_inf->dN.at(p-2)(i+1) / right_denom;
                                
                derivative_value = p * (derivative_left_term - derivative_right_term);
                derivative_temp_basic_function(i) = derivative_value;  //Calculate ddNi,p             
            }
            spline_inf->ddN.push_back(derivative_temp_basic_function); 
        }
    }
}

geometry_msgs::Point Curve_common::CalculateDerivativeCurvePoint(Spline_Inf *spline_inf, double u_data, int differential_times, bool UsingNURBS)
{
    geometry_msgs::Point derivative_curve_point;
    int p_degree = spline_inf->order - 1;
    double sum_x = 0, sum_y = 0;
    double sum_denom = 0;
    double sum_derivative_once_denom = 0;
    double sum_derivative_twice_denom = 0;
    double R_fraction = 0;
    double dR_left_term_fraction = 0;
    double dR_right_term = 0;
    double ddR_left_term_fraction = 0;
    double ddR_median_term_fraction = 0;
    double ddR_right_term_fraction = 0;

    spline_inf->R_double_vec.clear();
    spline_inf->dR_double_vec.clear();
    spline_inf->ddR_double_vec.clear();

    //Debug use
    // std::cout << "N_double_vec :" << "\n";
    // for(int i = 0; i < spline_inf->N_double_vec.size(); i++)
    // {
    //     std::cout << spline_inf->N_double_vec.at(i) << " ";
    // }
    // std::cout << "\n";

    // std::cout << "dN :" << "\n";
    // for(int i = 0; i < spline_inf->dN.back().size(); i++)
    // {
    //     std::cout << spline_inf->dN.back()(i) << " ";
    // }
    // std::cout << "\n";

    //TODO: Safety check
    if(UsingNURBS == false)
    {
        if(differential_times == 1)
        {
            for(int i = 0; i < spline_inf->control_point.size(); i++)
            { 
                sum_x += spline_inf->control_point.at(i)(0) * spline_inf->dN.back()(i);
                sum_y += spline_inf->control_point.at(i)(1) * spline_inf->dN.back()(i); 
                // std::cout << i << "'s spline_inf.dN : " << spline_inf->dN.back()(i) << "\n";
            }
        }
        if(differential_times == 2)
        {
            for(int i = 0; i < spline_inf->control_point.size(); i++)
            { 
                sum_x += spline_inf->control_point.at(i)(0) * spline_inf->ddN.back()(i);
                sum_y += spline_inf->control_point.at(i)(1) * spline_inf->ddN.back()(i); 
                // std::cout << i << "'s spline_inf.ddN : " << spline_inf->ddN.back()(i) << "\n";
            }
        }
    }
    else
    {        
        //TODO: Safety check differential_times
        for(int i = 0; i < spline_inf->control_point.size(); i++)
        { 
            sum_denom += spline_inf->N_double_vec.at(i) * spline_inf->weight.at(i);
            sum_derivative_once_denom += spline_inf->dN.back()(i) * spline_inf->weight.at(i);
            if(differential_times == 2)
                sum_derivative_twice_denom += spline_inf->ddN.back()(i) * spline_inf->weight.at(i);
        }
        for(int i = 0; i < spline_inf->control_point.size(); i++)
        { 
            if(sum_denom != 0)
            {
                R_fraction = spline_inf->N_double_vec.at(i) * spline_inf->weight.at(i);

                dR_left_term_fraction = spline_inf->dN.back()(i) * spline_inf->weight.at(i);
                dR_right_term = R_fraction * sum_derivative_once_denom / std::pow(sum_denom, 2);

                if(differential_times == 2)
                {
                    ddR_left_term_fraction = spline_inf->ddN.back()(i) * spline_inf->weight.at(i);                
                    ddR_median_term_fraction = 2 * dR_left_term_fraction * sum_derivative_once_denom + R_fraction * sum_derivative_twice_denom;
                    ddR_right_term_fraction = 2 * R_fraction * std::pow(sum_derivative_once_denom, 2);
                    spline_inf->ddR_double_vec.push_back( (ddR_left_term_fraction / sum_denom) - (ddR_median_term_fraction / std::pow(sum_denom, 2)) + (ddR_right_term_fraction / std::pow(sum_denom, 3)) );
                }
                
                spline_inf->R_double_vec.push_back(R_fraction / sum_denom);
                spline_inf->dR_double_vec.push_back((dR_left_term_fraction / sum_denom) - dR_right_term);                
            }
            else
            {   
                spline_inf->R_double_vec.push_back(0);
                spline_inf->dR_double_vec.push_back(0);
                spline_inf->ddR_double_vec.push_back(0);
            }
        }

        if(differential_times == 1)
        {
            for(int i = 0; i < spline_inf->control_point.size(); i++)
            { 
                sum_x += spline_inf->control_point.at(i)(0) * spline_inf->dR_double_vec.at(i);
                sum_y += spline_inf->control_point.at(i)(1) * spline_inf->dR_double_vec.at(i); 
                //std::cout << i << "'s spline_inf.dN : " << spline_inf->dN.back()(i) << "\n";
            }
        }
        if(differential_times == 2)
        {
            for(int i = 0; i < spline_inf->control_point.size(); i++)
            { 
                sum_x += spline_inf->control_point.at(i)(0) * spline_inf->ddR_double_vec.at(i);
                sum_y += spline_inf->control_point.at(i)(1) * spline_inf->ddR_double_vec.at(i); 
            }
        }
    }
  
    derivative_curve_point.x = sum_x;
    derivative_curve_point.y = sum_y;
    derivative_curve_point.z = 0;

    //Debug use
    // std::cout << u_data << "'s derivative current_pose x : " << derivative_curve_point.x << "\n";
    // std::cout << u_data << "'s derivative current_pose y : " << derivative_curve_point.y << "\n";

    return derivative_curve_point;
}

nav_msgs::Path Curve_common::Generate_DerivativeBsplineCurve(Spline_Inf bspline_inf, int differential_times, double t_intervel, std::string frame_id)
{
    geometry_msgs::Point derivative_point_result;
    nav_msgs::Path bspline_derivative_result;
    geometry_msgs::PoseStamped current_pose;
    
    bspline_derivative_result.header.frame_id = frame_id;
    bspline_derivative_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = frame_id;

    int p_degree = bspline_inf.order - 1;
    int n = bspline_inf.control_point.size() - 1;
    //TODO: Check knot vector size and sequence is correect
    int m = bspline_inf.knot_vector.size() - 1; //The last knot vector index number
    int segment = 1 / t_intervel;
    double delta_t = 0;
    double curve_parameter = 0;
    std::vector<double> curve_parameter_vec;

    //TODO: Can optimal this part code
    delta_t = 1 / (double)(segment - 1);
    curve_parameter_vec.resize(segment);
    for(int u = 0; u < segment; u++)
    {
        curve_parameter_vec[u] = curve_parameter;
        curve_parameter += delta_t;
    }
    
    //TODO: Can support high order derivative in loop
    for(int u = 0; u < segment; u++)
    {
        CalculateDerivativeBasisFunc(&bspline_inf, curve_parameter_vec[u], differential_times);
        derivative_point_result = CalculateDerivativeCurvePoint(&bspline_inf, curve_parameter_vec[u], differential_times, false);
        current_pose.header.seq = u;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = derivative_point_result.x;
        current_pose.pose.position.y = derivative_point_result.y;
        bspline_derivative_result.poses.push_back(current_pose);
    }

    return bspline_derivative_result;
}

nav_msgs::Path Curve_common::Generate_DerivativeBasisFuncCurve(Spline_Inf bspline_inf, int differential_times, int index, double t_intervel, std::string frame_id)
{
    geometry_msgs::Point derivative_point_result;
    nav_msgs::Path derivative_basis_result;
    geometry_msgs::PoseStamped current_pose;
    
    derivative_basis_result.header.frame_id = frame_id;
    derivative_basis_result.header.stamp = ros::Time::now();
    current_pose.header.frame_id = frame_id;

    //TODO: Add error ckeck function(below code with error)
    // if(differential_times == 0 && index < bspline_inf.N_double_vec.size())
    // {
    //     std::cout << "Without this index's basis function, vector size is :" << bspline_inf.N_double_vec.size() << "\n";
    //     return derivative_basis_result;
    // }
    // if(differential_times == 1 && index < bspline_inf.dN.back().size())
    // {
    //     std::cout << "Without this index's basis function, vector size is :" << bspline_inf.dN.back().size() << "\n";
    //     return derivative_basis_result;
    // }
    // if(differential_times == 2 && index < bspline_inf.ddN.back().size())
    // {
    //     std::cout << "Without this index's basis function, vector size is :" << bspline_inf.ddN.back().size() << "\n";
    //     return derivative_basis_result;
    // }
    
    int p_degree = bspline_inf.order - 1;
    int n = bspline_inf.control_point.size() - 1;
    //TODO: Check knot vector size and sequence is correect
    int m = bspline_inf.knot_vector.size() - 1; //The last knot vector index number
    int segment = 1 / t_intervel;
    double delta_t = 0;
    double curve_parameter = 0;
    std::vector<double> curve_parameter_vec;

    //TODO: Can optimal this part code
    delta_t = 1 / (double)(segment - 1);
    curve_parameter_vec.resize(segment);
    for(int u = 0; u < segment; u++)
    {
        curve_parameter_vec[u] = curve_parameter;
        curve_parameter += delta_t;
    }
    
    //TODO: Can support high order derivative in loop
    for(int u = 0; u < segment; u++)
    {
        CalculateDerivativeBasisFunc(&bspline_inf, curve_parameter_vec[u], differential_times);
        current_pose.header.seq = u;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = curve_parameter_vec[u];
        if(differential_times == 0)
            current_pose.pose.position.y = bspline_inf.N_double_vec.at(index);
        else if(differential_times == 1)
            current_pose.pose.position.y = bspline_inf.dN.back()(index);
        else
            current_pose.pose.position.y = bspline_inf.ddN.back()(index);
        derivative_basis_result.poses.push_back(current_pose);
    }

    return derivative_basis_result;
}

double Curve_common::CalculateCurvature(Spline_Inf spline_inf, double u_data, bool UsingNURBS)
{
    if(u_data > 1 || u_data < 0)
    {
        std::cout << "Error, u_data need between 0 and 1" << "\n";
        return 0;
    }

    Eigen::Vector3d derivative_once_point;
    Eigen::Vector3d derivative_twice_point;

    double curvature = 0;
    double fraction = 0;
    double denominator = 0;

    CalculateDerivativeBasisFunc(&spline_inf, u_data, 2);
    derivative_once_point = EigenVecter3dFromPointMsg(CalculateDerivativeCurvePoint(&spline_inf, u_data, 1, UsingNURBS));
    derivative_twice_point = EigenVecter3dFromPointMsg(CalculateDerivativeCurvePoint(&spline_inf, u_data, 2, UsingNURBS));
    
    fraction = derivative_once_point.cross(derivative_twice_point).lpNorm<2>();
    denominator = std::pow(derivative_once_point.lpNorm<2>(), 3);

    if(denominator == 0)
        return curvature = 0;
    else
        return curvature = fraction / denominator;
}

double Curve_common::CalculateCurvatureRadius(Spline_Inf spline_inf, double u_data, bool UsingNURBS)
{
    double curvature_radius = 0;
    double curvature = 0;
    
    curvature = CalculateCurvature(spline_inf, u_data, UsingNURBS);

    if(curvature == 0)
        return curvature_radius = 0;
    else
        return curvature_radius = 1 / curvature;
}

double Curve_common::CalculateCurveLength(Spline_Inf spline_inf, double start_u, double end_u, int sub_intervals, bool UsingNURBS)
{
    if(sub_intervals % 2 != 0)
    {
        std::cout << "Error! sub_intervals value is a even number" << "\n";
        return 0;
    }

    Eigen::Vector3d eigen_derivative_point;
    double sum_length = 0;
    double interval = 0;
    double u_index = 0;

    interval = (end_u - start_u) / (double)sub_intervals;
    
    for(int i = 0; i <= sub_intervals; i++)
    {
        u_index = start_u + interval * i;
        CalculateDerivativeBasisFunc(&spline_inf, u_index, 1);
        eigen_derivative_point = EigenVecter3dFromPointMsg(CalculateDerivativeCurvePoint(&spline_inf, u_index, 1, UsingNURBS));

        if(i == 0 || i == sub_intervals)
            sum_length += eigen_derivative_point.lpNorm<2>();
        else if((i % 2) == 1)
            sum_length += 4 * eigen_derivative_point.lpNorm<2>();
        else if((i % 2) == 0)
            sum_length += 2 * eigen_derivative_point.lpNorm<2>();
    }

    sum_length *= interval / 3;
    
    return sum_length;
}