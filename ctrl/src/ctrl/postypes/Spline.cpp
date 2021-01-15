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
    this->counter = _jv["counter"].asDouble();


    this->x = _jv1["x1"].asDouble();
    this->y = _jv1["y1"].asDouble();
    this->z = _jv1["z1"].asDouble();

}

void Spline::get_value()
{

  //  std::cout << spline_vec.at(0).at(0) << " " << spline_vec.at(1).at(0) << " " << spline_vec.at(2).at(0) <<" " <<  std::endl;
}

double Spline::get_x(){
    return this->x;
}

double Spline::get_y(){
    return this->y;
}

double Spline::get_z(){
    return this->z;
}

//double Spline::get_counter(){
//    this->counter;
//    return counter;
//}

void Spline::write_toVector()
{
    spline_vec.push_back(x_vec);
    spline_vec.push_back(y_vec);
    spline_vec.push_back(z_vec);
    std::cout << spline_vec.at(0).at(0) << " " << spline_vec.at(1).at(0) << " " << spline_vec.at(2).at(0) <<" " <<  std::endl;
    std::cout << spline_vec.at(0).at(1) << " " << spline_vec.at(1).at(1) << " " << spline_vec.at(2).at(1) <<" " <<  std::endl;

}
//

