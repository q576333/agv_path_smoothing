#include "agv_path_smoothing/Curve_common.h"

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
        std::cout << u << "'s current_pose x : " << sum_x << "\n";
        std::cout << u << "'s current_pose y : " << sum_y << "\n";
        current_pose.header.seq = u;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = sum_x;
        current_pose.pose.position.y = sum_y;
        bspline_curve_result.poses.push_back(current_pose);        
    }

    std::cout << "End calculate bspline function" << "\n";
    return bspline_curve_result;
}