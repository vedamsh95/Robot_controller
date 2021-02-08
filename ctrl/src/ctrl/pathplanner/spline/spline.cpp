#define _USE_MATH_DEFINES

#include "spline.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <SixDPos.h>
#include "../../sdir_ctrl.h"
#include "../../kinematics/direct/fw_kinematics.h"
#include "../../kinematics/inverse/inverse_kinematics.h"
#include "cassert"

#define PI 3.14159265
#define max_acc 300.0
#define max_vel 115.0

/*
* multiplication of a vector with a constant value
* 
* @param k {@ref double} constant factor
* @param vec {@ref vector<double>} vector to be multiplied with the constant
* @return {@ref vector<double>} vector multiplied with constant
*/
vector<double> konst_mult(double k, vector<double> vec)
{
    for (int i = 0; i < vec.size(); ++i)
    {
        vec[i] = vec[i] * k;
    }
    return vec;
}

/*
* summation of two vector of same size; vec1 + vec2
* 
* @param vec1 {@ref vector<double>} first vector
* @param vec2 {@ref vector<double>} second vector
* @return {@ref vector<double>} vector after summation
*/
vector<double> add_vec(vector<double> vec1, vector<double> vec2)
{
    assert(vec1.size() == vec2.size());
    for (int i = 0; i < vec1.size(); ++i)
    {
        vec1[i] = vec1[i] + vec2[i];
    }
    return vec1;
}
/*
* subtraction of two vectors of same size; vec1 - vec2
* 
* @param vec1 {@ref vector<double>} first vector; vector to be subtracted from
* @param vec2 {@ref vector<double>} second vector; vector to subtract
* @return {@ref vector<double>} vector after subtraction
*/
vector<double> sub_vec(vector<double> vec1, vector<double> vec2)
{
    assert(vec1.size() == vec2.size());
    for (int i = 0; i < vec1.size(); ++i)
    {
        vec1[i] = vec1[i] - vec2[i];
    }
    return vec1;
}

/*
* calculate the quintic bezier spline for a given start-point, -tangent and -acceleration, as well as given end-point, -tangent and acceleration at position t
* 
* @param t {@ref double} value between 0 and 1; describes which point on the spline shall be calculated
* @param p_0, t_s, a_s {@ref double} start-point, -tangent, -accerleration
* @param p_5, t_e, a_e {@ref double} end-point, tangent, acceleration
* @return {@ref vector<double>} point on the spline at the time t
*/
vector<double> quintic_bezier_spline(double t, vector<double> t_s, vector<double> a_s, vector<double> t_e, vector<double> a_e, vector<double> p_0, vector<double> p_5)
{
    // calculate control points
    vector<double> p_1 = add_vec(konst_mult(1 / 5, t_s), p_0);
    vector<double> p_2 = sub_vec(add_vec(konst_mult(1 / 20, a_s), konst_mult(2, p_1)), p_0);
    vector<double> p_4 = sub_vec(p_5, konst_mult(1 / 5, t_e));
    vector<double> p_3 = sub_vec(add_vec(konst_mult(1 / 20, a_e), konst_mult(2, p_4)), p_5);
    double t_inv = 1 - t;
    vector<double> s = add_vec(add_vec(add_vec(add_vec(add_vec(konst_mult(pow(t_inv, 5), p_0)
        ,konst_mult(5 * pow(t_inv, 4) * t, p_1))
        ,konst_mult(10 * pow(t_inv, 3) * pow(t, 2), p_2))
        ,konst_mult(10 * pow(t_inv, 2) * pow(t, 3), p_3))
        ,konst_mult(5 * t_inv * pow(t, 4), p_4))
        ,konst_mult(pow(t, 5), p_5));
    return s;
}

/*
* calculate the distances between the neighbouring points if multiple points are used for the spline
* 
* @param p {@ref vector<vector<double>>} vector of the pionts, which are vectors of doubles
* @return {@ref vector<double>} vector of the distances between the points
*/
vector<double> calculate_distance(vector<vector<double>> p)
{
    vector<double> distances;
    auto size = p.size();
    for (int i = 0; i < size; i++)
    {
        distances[i] = sqrt(pow((p[i][0] - p[i + 1][0]), 2)
            + pow((p[i][1] - p[i + 1][1]), 2)
            + pow((p[i][2] - p[i + 1][2]), 2));
    }

    return distances;
}

/*
* calculate the bounds if spline is calculated for multiple points
* 
* @param distances {@ref vector<double>} vector of the distances between the different points used for the spline
* @return {@ref vector<double>} returns the bounds for the given distances as a vector
*/
vector<double> calculate_bounds(vector<double> distances)
{
    vector<double> bounds;
    bounds[0] = 0;
    auto size = distances.size();
    for (int j = 0; j <= size; j++)
    {
        bounds[j + 1] = bounds[j] + distances[j];
    }
    return bounds;
}
/*
* choose the closest configuration out of the multiple configurations given by inverse kinematic
* 
* @param last_config {@ref Configuration} configuration of the last step
* @param config {@ref vector<Configuration*>*} pointer on the vector of the possible solutions form inverse kinematics
* @return {@Configuration} next configuration of the robot s.t. the distance between the configurations is minimal
*/
Configuration get_next_config(Configuration last_config, vector<Configuration*>* config)
{
    Configuration closest;
    vector<Configuration*> config_vector = *config;
    double min_distance = INFINITY;
    Configuration next_config;

    for (int i = 0; i <= config->size(); i++)
    {
        Configuration* config_i_pointer = config_vector[i];
        Configuration config_value = *config_i_pointer;
        double distance = sqrt(pow((last_config[0] - config_value[0]), 2)
            + pow((last_config[1] - config_value[1]), 2) 
            + pow((last_config[2] - config_value[2]), 2)
            + pow((last_config[3] - config_value[3]), 2)
            + pow((last_config[4] - config_value[4]), 2)
            + pow((last_config[5] - config_value[5]), 2)
        );

        if (distance < min_distance)
        {
            min_distance = distance;
            next_config = config_value;
        }

    }

    return next_config;
}
 /*
 *calculate the trajectory for a spline
 *  currently only implemented for one end-point, but cannot be reached due to some error in the main.cpp
 * 
 * @param _start_cfg {@ref Configuration} current configuration of the robot
 * @param _end_pos {@ref SixDPos} final position of the robot
 * @return {@ref Trajectory} trajectory form start configuration to the end position
 */
Trajectory* Spline::get_spline_trajectoy(Configuration* _start_cfg, SixDPos* _end_pos) {    

    /*
    double j_0 = 0;
    double j_1 = - M_PI_2;
    double j_2 = M_PI_2;
    double j_3 = 0;
    double j_4 = 0;
    double j_5 = 0;
    Configuration start_config = new Configuration{ array<double, 6>{ j_0, j_1, j_2, j_3, j_4, j_5 } };
    Configuration* start_config_pointer = &start_config;
    */

    // initializing the start-point, -tangent and -acceleration of the spline
    FwKinematics fwkinematics_spline;
    SixDPos point_start = fwkinematics_spline.get_fw_kinematics(_start_cfg);
    vector<double> p_s = { point_start[0], point_start[1], point_start[2] };
    vector<double> t_s{ 0, 0, 0 };
    vector<double> a_s{ 0, 0, 0 };
    double a_1 = point_start[3];
    double b_1 = point_start[4];
    double c_1 = point_start[5];

    /*
    double x_2 = 1000;
    double y_2 = 1500;
    double z_2 = 1000;
    */

    // initializing the end-point, -tangent, and -acceleration of the spline
    SixDPos end_position = *_end_pos;
    vector<double> p_e{ end_position[0], end_position[1], end_position[2] };
    vector<double> t_e{ 0, 0, 0 };
    vector<double> a_e{ 0, 0, 0 };

    // initializing the trajectory
    Trajectory* trajectory = new Trajectory();
    vector<Configuration*> trajectory_data_pointer;
    vector<Configuration> trajectory_data;

    // calculate every 0.004 the next point for the trajectory
    for (double i = 0; i <= 1; i = i + 0.004)
    {
        // get the point on the spline
        vector<double> point = quintic_bezier_spline(i, t_s, a_s, t_e, a_e, p_s, p_e);

        // get robot configuration for the next point via inverse kinematics
        SixDPos p_6D = new SixDPos(point[0], point[1], point[2], a_1, b_1, c_1);
        SixDPos* point_6D = &p_6D;
        InvKinematics invkinematics_spline;
        vector<Configuration*>* config = new vector<Configuration*>();
        config = invkinematics_spline.get_inv_kinematics(point_6D);

        // choose the closest configuration to the previous one
        Configuration prev_config;
        Configuration next_config;
        if (i == 0)
        {
            prev_config = *_start_cfg;
        }
        else
        {
            prev_config = next_config;
        }

        // set the trajectory data
        trajectory_data[i] = get_next_config(prev_config, config);
        trajectory_data_pointer[i] = &trajectory_data[i];
    }

    // return trajectory
    trajectory->set_trajectory(trajectory_data_pointer);
    return trajectory;
}