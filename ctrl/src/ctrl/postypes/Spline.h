//
// Created by Timo on 14.01.2021.
//

#ifndef SDIR_CTRL2020_SPLINE_H
#define SDIR_CTRL2020_SPLINE_H
#include "json.h"

#endif //SDIR_CTRL2020_SPLINE_H

class Spline {
public:
    std::vector<double> vec;
    double x,y,z;
    //tesing for splines
   Spline(Json::Value _jv,Json::Value _jv1);
    void deserialize_from_json_config(Json::Value _jv,Json::Value _jv1);
    void get_value();

};
