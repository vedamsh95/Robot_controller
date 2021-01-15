//
// Created by Timo on 14.01.2021.
//
#include "Spline.h"
#include <iostream>

// testing for splines
 Spline::Spline(Json::Value _jv, Json::Value _jv1){
    this->deserialize_from_json_config(_jv, _jv1);
}
void Spline::deserialize_from_json_config(Json::Value _jv, Json::Value _jv1){
    std::cout << "counter" << _jv["counter"].asDouble() <<std::endl;
    
    this->x = _jv1["x1"].asDouble();
    this->y = _jv1["y1"].asDouble();
    this->z = _jv1["z1"].asDouble();
}

void Spline::get_value()
{
    std::cout << this->x << " " << this->y << " " << this->z <<" " << std::endl;
}

