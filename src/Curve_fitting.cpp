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
        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = i / (double)n;
        }    
    }
    else if(parameter_method == dis_u_method::Chord)
    {
        for(int i = 0; i <= n - 1; i++)
        {
            chord_length = std::sqrt( std::pow(input_point->at(i+1).position[0] - input_point->at(i).position[0], (double)2.0) + std::pow(input_point->at(i+1).position[1] - input_point->at(i).position[1], (double)2.0));
            sum_length += chord_length;
            chord_length_vec.push_back(chord_length);         
        }
        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = input_point->at(i - 1).u_data + chord_length_vec[i - 1] / sum_length;
        }
    }
    else if(parameter_method == dis_u_method::Centripetal)
    {
        for(int i = 0; i <= n - 1; i++)
        {
            chord_length = std::sqrt(std::sqrt( std::pow(input_point->at(i+1).position[0] - input_point->at(i).position[0], (double)2.0) + std::pow(input_point->at(i+1).position[1] - input_point->at(i).position[1], (double)2.0)));
            sum_length += chord_length;
            chord_length_vec.push_back(chord_length);            
        }

        for(int i = 1; i <= n - 1; i++)
        {
            input_point->at(i).u_data = input_point->at(i - 1).u_data + chord_length_vec[i - 1] / sum_length;
        }
    }
    else
    {
        //TODO: Error message need modify to correct position
        std::cout << "Please choice calculation discreate point parameter method" << "\n";
        return;
    }
}

void Curve_fitting::CalculateKnotVector(EigenTrajectoryPoint::Vector input_point, Spline_Inf *curvefit_inf, knotvector_method knot_method)
{
    double sum_u = 0;
    int n = input_point.size() - 1;
    int p_degree = curvefit_inf->order - 1;
    int m = input_point.size() + p_degree;

    curvefit_inf->knot_vector.clear();
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
    else if(knot_method == knotvector_method::LimitDerivative_Average)
    {
        m += 2; //+2 for add two new control points
        curvefit_inf->knot_vector.clear();
        curvefit_inf->knot_vector.resize(m+1);
        for(int i = 0; i <= m; i++)
        {
            if(i <= p_degree)
                curvefit_inf->knot_vector.at(i) = 0;
            else if (i > p_degree && i < m - p_degree)
            {
                sum_u = 0;
                for(int j = i - p_degree - 1; j <= i - 2; j++)
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
        std::cout << "Please choice calculation knot vector method" << "\n";
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
        }   
        curvefit_result.N.push_back(temp_basic_function);
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
        }
    }

    //Transpose curvefit_result.curvefit_N by itself
    curvefit_result.curvefit_N.transposeInPlace();

    //TODO: Need consider curvefit_N matrix is not a square matrix
    control_point = curvefit_result.curvefit_N.inverse() * discreate_point;

    //Transform control point result from Eigen matrix to vector
    curvefit_result.control_point.resize(n+1);
    for(int i = 0; i <= n; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            curvefit_result.control_point.at(i)(j) = control_point(i, j);
        }
    }

    std::cout << "End unlimit curve fitting" << "\n";
    return curvefit_result;
}

Spline_Inf Curve_fitting::LimitCurveFitting(EigenTrajectoryPoint::Vector input_point, int order, std::vector<geometry_msgs::Point> derivatives_spec, dis_u_method parameter_method, knotvector_method knot_method, double start_vector_weight, double end_vector_weight)
{
    Spline_Inf curvefit_result;
    curvefit_result.order = order;
    
    if(derivatives_spec.size() != 2)
    {
        std::cout << "Please specified the initial and final derivative vectors" << "\n";
        return curvefit_result;
    }

    //TODO: Maybe can add new CalculateKnotVector with limit derivative function
    if(knot_method != knotvector_method::LimitDerivative_Average)
    {
        std::cout << "Please use LimitDerivative_Average method when you want use LimitCurveFitting function" << "\n";
        return curvefit_result;
    }

    int n = input_point.size() - 1; 
    int p_degree = order - 1;
    int m = input_point.size() + p_degree + 2; //+2 for add two new control points
    int index_offset = 0;
    double left_denom = 0, right_denom = 0;
    double left_term = 0, right_term = 0;
    double sum_x = 0, sum_y = 0;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > temp_basic_function_eigen;
    Eigen::VectorXd temp_basic_function;
    Eigen::MatrixXd control_point;
    Eigen::MatrixXd discreate_point;
    Eigen::MatrixXd solve_discreate_point;

    CalculateDiscreatePointParameter(&input_point, parameter_method);
    CalculateKnotVector(input_point, &curvefit_result, knot_method);

    //TODO: Can optimal this part code
    //Transform discreate point to Eigen matrix (not require)
    discreate_point.resize(n+1+2, 2);
    solve_discreate_point.resize(n+1-2, 2);
    for(int i = 0; i <= n + 2; i++)
    {
        if(i == 1)
        {
            index_offset++;
            discreate_point(i, 0) = derivatives_spec.at(0).x * (curvefit_result.knot_vector.at(p_degree + 1) / (double)p_degree) * start_vector_weight;
            discreate_point(i, 1) = derivatives_spec.at(0).y * (curvefit_result.knot_vector.at(p_degree + 1) / (double)p_degree) * start_vector_weight;
        
        }
        else if(i == ((n+2) - 1))
        {
            index_offset++;
            discreate_point(i, 0) = derivatives_spec.at(1).x * ((1 - curvefit_result.knot_vector.at(m - p_degree - 1)) / (double)p_degree) * end_vector_weight;
            discreate_point(i, 1) = derivatives_spec.at(1).y * ((1 - curvefit_result.knot_vector.at(m - p_degree - 1)) / (double)p_degree) * end_vector_weight;
        }
        else
        {
            for(int j = 0; j < 2; j++)
            {
                discreate_point(i, j) = input_point.at(i - index_offset).position(j);
            }    
        }
    }

    //Debug use
    // for(int i = 0; i <= m; i++)
    // {
    //     std::cout << i << "'s index of knot vector value: " << curvefit_result.knot_vector.at(i) << "\n";
    // }

    //Calculate degree = 0's basic function
    curvefit_result.N.resize(m);
    temp_basic_function.resize(n+1);
    for(int i = 0; i < m; i++)
    {
        for(int point_index = 0; point_index <= n; point_index++)
        {
            if(curvefit_result.knot_vector.at(i) != curvefit_result.knot_vector.at(n+2))
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
        }   
        curvefit_result.N.at(i) = temp_basic_function;
    }

    //Calculate the rest of basic function
    temp_basic_function_eigen.resize(m - 1);
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
                // if(p == p_degree)
                //     std::cout << i << "'s " << p << "'s " << "index: " << point_index << " last basic function:" << temp_basic_function(point_index) << "\n";      
            }
            temp_basic_function_eigen.push_back(temp_basic_function);
        }
        curvefit_result.N = temp_basic_function_eigen;
    }

    //Extract matrix(N) from median basic function (NP = Q)
    int orignal_size = n+1;
    //int curvefit_N_matrix_size = n+1+2;
    curvefit_result.curvefit_N = Eigen::MatrixXd::Zero(orignal_size - 2, orignal_size - 2);
    Eigen::MatrixXd curvefit_N_temp = Eigen::MatrixXd::Zero(orignal_size, n + 2 + 1);
    for(int point_index = 0; point_index <= n; point_index++)
    {
        for(int i = 0; i <= n + 2; i++)
        {
            curvefit_N_temp(i, point_index) = curvefit_result.N.at(i)(point_index);      
        }
    }
    curvefit_N_temp.transposeInPlace();
    curvefit_result.curvefit_N = curvefit_N_temp.block(1, 2, orignal_size - 2, orignal_size - 2); //median block

    //TODO: Adding high order limit fitting support
    //Preparing the discerate point(Q) be a right term (NP = Q)
    for(int i = 1; i <= n - 1; i++)
    {   
        if(i == 1)
        {
            solve_discreate_point(i - 1, 0) = discreate_point(2, 0) - curvefit_N_temp(1, 1) * (discreate_point(1, 0) + discreate_point(0, 0));
            solve_discreate_point(i - 1, 1) = discreate_point(2, 1) - curvefit_N_temp(1, 1) * (discreate_point(1, 1) + discreate_point(0, 1));
        }
        else if(i == n - 1)
        {
            // solve_discreate_point(i - 1, 0) = discreate_point(n, 0) - curvefit_N_temp(n, n) * (discreate_point(n+2, 0) - discreate_point(n+1, 0));
            // solve_discreate_point(i - 1, 1) = discreate_point(n, 1) - curvefit_N_temp(n, n) * (discreate_point(n+2, 1) - discreate_point(n+1, 1));
            solve_discreate_point(i - 1, 0) = discreate_point(n, 0) - (1 - (curvefit_N_temp(n-1, n - 1) + curvefit_N_temp(n-1, n))) * (discreate_point(n+2, 0) - discreate_point(n+1, 0));
            solve_discreate_point(i - 1, 1) = discreate_point(n, 1) - (1 - (curvefit_N_temp(n-1, n - 1) + curvefit_N_temp(n-1, n))) * (discreate_point(n+2, 1) - discreate_point(n+1, 1));
            //std::cout << "(1 - (curvefit_N_temp(n, n - 1) + curvefit_N_temp(n, n))) : " << (1 - (curvefit_N_temp(n-1, n - 1) + curvefit_N_temp(n-1, n))) << "\n";
        }
        else 
        {
            solve_discreate_point(i - 1, 0) = input_point.at(i).position(0);
            solve_discreate_point(i - 1, 1) = input_point.at(i).position(1);
        }

    }

    //TODO: Delete debug code
    //Debug test
    // std::cout << "solve_discreate_point : " << "\n";
    // std::cout << solve_discreate_point << "\n";

    //TODO: insert derivative requirment to curvefit_result.curvefit_N eigen matrix in this scope
    //Transform basic function to Eigen matrix
    //curvefit_result.curvefit_N.resize(n+1+2, n+1+2);
    
    //Eigen::VectorXd derivate_N_vec = Eigen::VectorXd::Ones(curvefit_N_matrix_size, curvefit_N_matrix_size);
    // std::cout << "derivate_N_vec : " << "\n";
    // std::cout << derivate_N_vec.block(0, 0, 1, curvefit_N_matrix_size) << "\n";
    
    // std::cout << "curvefit_N_temp first block matrix : " << "\n";
    // std::cout << curvefit_N_temp.topRows<1>() << "\n";

    // std::cout << "curvefit_N_temp bottom left column : " << "\n";
    // std::cout << curvefit_N_temp.block(1, 0, orignal_size - 2, 1) << "\n";

    // std::cout << "curvefit_N_temp top right column : " << "\n";
    // std::cout << curvefit_N_temp.block(1, orignal_size - 1, orignal_size - 2, 1) << "\n";

    // std::cout << "curvefit_N_temp median block matrix : " << "\n";
    // std::cout << curvefit_N_temp.block(1, 2, orignal_size - 2, orignal_size - 2) << "\n";

    // std::cout << "curvefit_N_temp last block matrix : " << "\n";
    // std::cout << curvefit_N_temp.bottomRows<1>() << "\n";

    //orignial one
    // curvefit_result.curvefit_N.block(0, 0, 1, orignal_size) = curvefit_N_temp.topRows<1>();
    // curvefit_result.curvefit_N.block(2, 0, orignal_size - 2, 1) = curvefit_N_temp.block(1, 0, orignal_size - 2, 1);
    // curvefit_result.curvefit_N.block(2, curvefit_N_matrix_size - 1, orignal_size - 2, 1) = curvefit_N_temp.block(1, orignal_size - 1, orignal_size - 2, 1);
    // //curvefit_result.curvefit_N.block(curvefit_N_matrix_size - 2, curvefit_N_matrix_size - 1, 1, orignal_size - 1) = curvefit_N_temp.topRightCorner(orignal_size - 1, 1);
    // curvefit_result.curvefit_N.block(2, 2, orignal_size - 2, orignal_size - 2) = curvefit_N_temp.block(1, 1, orignal_size - 2, orignal_size - 2); //median block
    // curvefit_result.curvefit_N.block(curvefit_N_matrix_size - 1, 2, 1, orignal_size) = curvefit_N_temp.bottomRows<1>();

    // curvefit_result.curvefit_N(1, 0) = -1;
    // curvefit_result.curvefit_N(1, 1) = 1;
    // curvefit_result.curvefit_N(curvefit_N_matrix_size - 2, curvefit_N_matrix_size - 2) = -1;
    // curvefit_result.curvefit_N(curvefit_N_matrix_size - 2, curvefit_N_matrix_size - 1) = 1;

    // curvefit_result.curvefit_N.block(1, 0, 1, 1) = derivate_N_vec.block(1, 1, 1, 1);
    // curvefit_result.curvefit_N.block(1, 1, 1, 1) = derivate_N_vec.block(1, 2, 1, 1);
    // curvefit_result.curvefit_N.block(curvefit_N_matrix_size - 2, curvefit_N_matrix_size - 2, 1, 1) = derivate_N_vec.block(1, 1, 1, 1);
    // curvefit_result.curvefit_N.block(curvefit_N_matrix_size - 1, curvefit_N_matrix_size - 1, 1, 1) = derivate_N_vec.block(1, 2, 1, 1);
    // std::cout << "curvefit_result.curvefit_N matrix : " << "\n";
    // std::cout << curvefit_result.curvefit_N << "\n";

    //Solve control point(P) (P = N^(-1)Q)
    //TODO: Need consider curvefit_N matrix is not a square matrix
    control_point.resize(n+1, 2);
    control_point = curvefit_result.curvefit_N.inverse() * solve_discreate_point;

    //Transform control point result from Eigen matrix to vector
    curvefit_result.control_point.resize(n+2+1);
    for(int i = 0; i <= n + 2; i++)
    {
        if(i == 0)
        {
            curvefit_result.control_point.at(i)(0) = discreate_point(0, 0);
            curvefit_result.control_point.at(i)(1) = discreate_point(0, 1);
            index_offset = 1;
        }
        else if(i == 1)
        {
            curvefit_result.control_point.at(i)(0) = discreate_point(1, 0) + discreate_point(0, 0);
            curvefit_result.control_point.at(i)(1) = discreate_point(1, 1) + discreate_point(0, 1);
            index_offset = 2;
        }
        else if(i == n + 2 - 1)
        {
            curvefit_result.control_point.at(i)(0) = -1 * discreate_point(n+1, 0) + discreate_point(n+2, 0);
            curvefit_result.control_point.at(i)(1) = -1 * discreate_point(n+1, 1) + discreate_point(n+2, 1);
        }
        else if(i == n + 2)
        {
            curvefit_result.control_point.at(i)(0) = discreate_point(n+2, 0);
            curvefit_result.control_point.at(i)(1) = discreate_point(n+2, 1);
        }
        else
        {
            for(int j = 0; j < 2; j++)
            {
                curvefit_result.control_point.at(i)(j) = control_point(i - index_offset, j);
            }
        }   
    }

    std::cout << "End limit curve fitting" << "\n";
    return curvefit_result;
}