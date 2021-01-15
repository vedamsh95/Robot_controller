//
// Created by Timo on 14.01.2021.
//

#ifndef SDIR_CTRL2020_SPLINE_H
#define SDIR_CTRL2020_SPLINE_H
#include "json.h"

#endif //SDIR_CTRL2020_SPLINE_H

class Spline {
public:
    std::vector<std::vector<double>> spline_vec;
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> z_vec;
    double x,y,z,a,b,c, counter;
    //tesing for splines
   Spline(Json::Value _jv,Json::Value _jv1);
    void deserialize_from_json_config(Json::Value _jv,Json::Value _jv1);
    void get_value();
    double get_counter();
    double get_x();
    double get_y();
    double get_z();
    void write_toVector();
    //

};
