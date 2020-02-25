#include "agv_path_smoothing/Curve_fitting.h"

#include <geometry_msgs/PoseStamped.h>

#include <iostream>

Curve_fitting::Curve_fitting()
{

}

void Curve_fitting::CalculateDiscreatePointParameter(EigenTrajectoryPoint::Vector *input_point, dis_u_method parameter_method)
{
    int n = input_point->size() - 1; //last point index
    double chord_length;
    std::vector<double> chord_length_vec;
    double sum_length = 0;

    //input_point->u_data.resize(n);
    input_point->at(0).u_data = 0.0;
    input_point->at(n).u_data = 1.0;
    
    if(parameter_method == dis_u_method::Average)
    {
        //std::cout << "input_point u_data is :" << "\n";
        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = i / (double)n;
            //std::cout << input_point->at(i).u_data << " ";
        }    
    }
    else if(parameter_method == dis_u_method::Chord)
    {
        for(int i = 0; i <= n - 1; i++)
        {
            chord_length = std::sqrt( std::pow(input_point->at(i+1).position[0] - input_point->at(i).position[0], (double)2.0) + std::pow(input_point->at(i+1).position[1] - input_point->at(i).position[1], (double)2.0));
            sum_length += chord_length;
            chord_length_vec.push_back(chord_length);   
            // std::cout << "chord_length :" << "\n";  
            // std::cout << chord_length << " ";       
        }
        //std::cout << "\n";
        //std::cout << "sum length :" << sum_length << "\n";

        //std::cout << "input_point u_data is :" << "\n";
        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = input_point->at(i - 1).u_data + chord_length_vec[i - 1] / sum_length;
            //std::cout << input_point->at(i).u_data << " ";
        }
        //std::cout << "\n";
    }
    else if(parameter_method == dis_u_method::Centripetal)
    {
        for(int i = 0; i <= n - 1; i++)
        {
            chord_length = std::sqrt(std::sqrt( std::pow(input_point->at(i+1).position[0] - input_point->at(i).position[0], (double)2.0) + std::pow(input_point->at(i+1).position[1] - input_point->at(i).position[1], (double)2.0)));
            sum_length += chord_length;
            chord_length_vec.push_back(chord_length);            
        }

        //std::cout << "sum length :" << sum_length << "\n";
        //std::cout << "input_point u_data is :" << "\n";
        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = input_point->at(i - 1).u_data + chord_length_vec[i - 1] / sum_length;
            std::cout << input_point->at(i).u_data << " ";
        }
        //std::cout << "\n";
    }
    else
    {
        //TODO: Error message need modify to correct position
        std::cout << "Please choice calculation discreate point parameter method";
        return;
    }
}

void Curve_fitting::CalculateKnotVector(EigenTrajectoryPoint::Vector input_point, Spline_Inf *curvefit_inf, knotvector_method knot_method)
{
    double sum_u = 0;
    int n = input_point.size() - 1;
    int p_degree = curvefit_inf->order - 1;
    int m = input_point.size() + p_degree;

    curvefit_inf->knot_vector.resize(m+1);
    //std::cout << "input_point knotvector is :" << "\n";
    if(knot_method == knotvector_method::Equal_space)
    {
        for(int i = 0; i <= m; i++)
        {
            if(i <= p_degree)
                curvefit_inf->knot_vector.at(i) = 0;
            else if (i > p_degree && i < m - p_degree)
                curvefit_inf->knot_vector.at(i) = (i - p_degree) / (double)(n - p_degree + 1);
            else
                curvefit_inf->knot_vector.at(i) = 1;     
            //std::cout << curvefit_inf->knot_vector.at(i) << " ";
        }
        //std::cout << "\n";
    }
    else if(knot_method == knotvector_method::Average)
    {
        for(int i = 0; i <= m; i++)
        {
            if(i <= p_degree)
                curvefit_inf->knot_vector.at(i) = 0;
            else if (i > p_degree && i < m - p_degree)
            {
                sum_u = 0;
                for(int j = i - p_degree; j <= i - 1; j++)
                {
                    sum_u += input_point.at(j).u_data;
                }

                curvefit_inf->knot_vector.at(i) = sum_u / p_degree;
            }                
            else
                curvefit_inf->knot_vector.at(i) = 1;            
        }
    }
    else
    {
        //TODO: Error message need modify to correct position
        std::cout << "Please choice calculation knot vector method";
        return;
    }
}

Spline_Inf Curve_fitting::UnLimitCurveFitting(EigenTrajectoryPoint::Vector input_point, int order, dis_u_method parameter_method, knotvector_method knot_method)
{
    Spline_Inf curvefit_result;
    curvefit_result.order = order;

    int n = input_point.size() - 1;
    int p_degree = order - 1;
    int m = input_point.size() + p_degree;
    double left_denom = 0, right_denom = 0;
    double left_term = 0, right_term = 0;
    double sum_x = 0, sum_y = 0;
    Eigen::VectorXd temp_basic_function;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > temp_basic_function_eigen;
    Eigen::MatrixXd control_point;
    Eigen::MatrixXd discreate_point;

    CalculateDiscreatePointParameter(&input_point, parameter_method);
    CalculateKnotVector(input_point, &curvefit_result, knot_method);

    control_point.resize(n+1, 2);
    discreate_point.resize(n+1, 2);
    curvefit_result.curvefit_N.resize(n+1, n+1);
    temp_basic_function.resize(n + 1);

    //Transform discreate point to Eigen matrix
    for(int i = 0; i <= n; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            discreate_point(i, j) = input_point.at(i).position(j);
        }            
    }

    //Debug use
    // for(int i = 0; i <= m; i++)
    // {
    //     std::cout << i << "'s index of knot vector value: " << curvefit_result.knot_vector.at(i) << "\n";
    // }

    //Calculate degree = 0's basic function
    for(int i = 0; i < m; i++)
    {
        for(int point_index = 0; point_index <= n ; point_index++)
        {
            if(curvefit_result.knot_vector.at(i) != curvefit_result.knot_vector.at(n))
            {
                if(curvefit_result.knot_vector.at(i) <= input_point.at(point_index).u_data && input_point.at(point_index).u_data < curvefit_result.knot_vector.at(i+1))                        
                    temp_basic_function(point_index) = 1;
                else
                    temp_basic_function(point_index) = 0;
            }
            else
            { 
                if(curvefit_result.knot_vector.at(i) <= input_point.at(point_index).u_data && input_point.at(point_index).u_data <= curvefit_result.knot_vector.at(i+1))                        
                    temp_basic_function(point_index) = 1;
                else
                    temp_basic_function(point_index) = 0;
            }
            //std::cout << temp_basic_function(point_index) << " ";
        }   
        curvefit_result.N.push_back(temp_basic_function);
        //std::cout << "\n";
    }

    //Calculate the rest of basic function
    for(int p = 1; p <= p_degree; p++)
    {
        temp_basic_function_eigen.clear();
        for(int i = 0; i < (m - p); i++)
        {
            for(int point_index = 0; point_index <= n ; point_index++)
            {
                left_denom = curvefit_result.knot_vector.at(p+i) - curvefit_result.knot_vector.at(i);
                right_denom = curvefit_result.knot_vector.at(i+p+1) - curvefit_result.knot_vector.at(i+1);

                if(left_denom == 0)
                    left_term = 0;
                else
                    left_term = (input_point.at(point_index).u_data - curvefit_result.knot_vector.at(i)) * curvefit_result.N.at(i)(point_index) / left_denom;

                if(right_denom == 0)
                    right_term = 0;
                else
                    right_term = (curvefit_result.knot_vector.at(i+p+1) - input_point.at(point_index).u_data) * curvefit_result.N.at(i+1)(point_index) / right_denom;
                
                temp_basic_function(point_index) = left_term + right_term;      

                //Debug use
                //std::cout << i << "'s " << p << "'s " << "index: " << point_index << " last basic function:" << temp_basic_function(point_index) << "\n";      
            }
            temp_basic_function_eigen.push_back(temp_basic_function);
        }
        curvefit_result.N = temp_basic_function_eigen;
    }

    //Transform basic function to Eigen matrix
    for(int point_index = 0; point_index <= n ; point_index++)
    {
        for(int i = 0; i <= n; i++)
        {
            curvefit_result.curvefit_N(i, point_index) = curvefit_result.N.at(i)(point_index);
            //std::cout << curvefit_result.curvefit_N(i, point_index) << " ";            
        }
        //std::cout << "\n";
    }

    //TODO: Check why inverse matrix result is transpose.
    // std::cout << "curvefit_result.curvefit_N inverse matrix : " << "\n";
    // std::cout << curvefit_result.curvefit_N.inverse() << "\n";

    //TODO: Need consider curvefit_N matrix is not a square matrix
    //control_point = curvefit_result.curvefit_N.inverse().colPivHouseholderQr().solve(discreate_point);
    control_point = curvefit_result.curvefit_N.inverse().transpose() * discreate_point;

    //Transform control point result from Eigen matrix to vector
    curvefit_result.control_point.resize(n+1);
    for(int i = 0; i <= n; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            curvefit_result.control_point.at(i)(j) = control_point(i, j);
        }
    }

    std::cout << "End fitting" << "\n";
    return curvefit_result;
}