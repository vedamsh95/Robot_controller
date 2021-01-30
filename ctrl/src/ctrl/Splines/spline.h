//
// Created by Timo on 07.01.2021.
//

#ifndef SDIR_CTRL2020_SPLINE_H
#define SDIR_CTRL2020_SPLINE_H


#include "Vector.h"
#include <vector>
#include <json.h>
#include "Trajectory.h"
#include <inverse_kinematics.h>
#include "ConfigProvider.h"

class Spline {

private:
    Vector<double,3> start_position, start_orientation;
    Vector<double, 6> start_config;
    double speed, acceleration;
    int num_points;
    std::vector<Vector<double, 3>> *points;
    double M_PII = 3.141592654;
    double timesteps;
    double marginpoint = ConfigProvider::getInstance().getmargin_point();
    double config_switch_limit = ConfigProvider::getInstance().getconfig_switch_limit();
    int config_switch_indent = -1;


public:
    //-------------------create vector to store acceleration and speed of every joint at every time point---------------
    std::vector<vector<double>> joint_velocities_vec;
    std::vector<Configuration*> config_vec;
    std::vector<Configuration*> temp_vec;

    Spline(Vector<double,3> start_point, Vector<double,3> start_orientation,std::vector<Vector<double, 3>> *points, double speed, double acceleration, Vector<double, 6> start_config_vec);
    void out();
    Trajectory* calculateSpline();

    double  calc_num(double num, int n);
    double calc_config_difference(Configuration* config1, Configuration* config2);
    Vector<double, 3> quintic_bezier_function(Vector<double, 3> point0, Vector<double, 3> point1, Vector<double, 3> point2, Vector<double, 3> point3,
                                              Vector<double, 3> point4, Vector<double, 3> point5, double t);
    bool checkconfiglimits(Configuration* config1, Configuration* config2,
                                   std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps);
    void add_middle_cfg(Configuration* config1, Configuration* config2,
                                         std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps);
    void add_middle_cfg2(Configuration* config1, Configuration* config2,
                                 std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps);


};


#endif //SDIR_CTRL2020_SPLINE_H
