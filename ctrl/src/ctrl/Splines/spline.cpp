//
// Created by Timo on 07.01.2021.
//

#include "spline.h"
#include "Vector.h"
#include <iostream>

Spline::Spline(Vector<double, 3> start_point, Vector<double, 3> start_orientation,
               std::vector<Vector<double, 3>> *points, double speed, double acceleration) {

    this->start_position[0] = start_point[0];
    this->start_position[1] = start_point[1];
    this->start_position[2] = start_point[2];

    this->start_orientation[0] = start_orientation[0];
    this->start_orientation[1] = start_orientation[1];
    this->start_orientation[2] = start_orientation[2];

    this->speed = speed;
    this->acceleration = acceleration;

    this->points = points;


}

void Spline::out() {
    std::cout << "Spline is defined: " << std::endl;
    std::cout << "Start Position: " << std::endl;
    start_position.output();
    std::cout << "Current Orientation " << std::endl;
    start_orientation.output();
    std::cout << "" << std::endl;
    std::cout << "Speed: " << speed <<std::endl;
    std::cout << "Acceleration: " << acceleration <<std::endl;
    std::cout << "Current Points are entered: " << std::endl;
    for(auto &tmp : *points){
        tmp.output();
    }

}

/*
Spline::Spline(double _t, double _ts, double _te, double _as, double _ae, Vector<double,3> start, Vector<double,3> end) {
    this->current_timestamp = 0;
    this->t =_t;
    this->t_s = _ts;
    this->t_e = _te;
    this->a_s = _as;
    this->a_e = _ae;

    this->p0 = start;
    this->p5 = end;

//-------------calculations -----------------//

    this->p1 = ((1/5)*_ts) + start;

    this->p2[0] = (1/20) * _as + 2.0*this->p1[0] -this->p0[0];
    this->p2[1] = (1/20) * _as + 2.0*this->p1[1] -this->p0[1];
    this->p2[2] = (1/20) * _as + 2.0*this->p1[2] -this->p0[2];

    this->p4[0] = end[0] -(1/5)*_te;
    this->p4[1] = end[1] -(1/5)*_te;
    this->p4[2] = end[2] -(1/5)*_te;

    this->p3[0] = (1/20) * _ae + 2*this->p4[0] - end[0];
    this->p3[1] = (1/20) * _ae + 2*this->p4[1] - end[1];
    this->p3[2] = (1/20) * _ae + 2*this->p4[2] - end[2];



}

double Spline::get_current_timestamp() const {
    return this->current_timestamp;
}

void Spline::recalc_timestamp() {
    this->current_timestamp += this->t;

}

double Spline::calc_num(double stemp, int n) {
    double result = (1-stemp);
    if(n == 0)
        return 1;
    else
        for(int i = 1; i<n; i++)
        {
            result *= (1-stemp);
        }
    return result;
}

std::vector<Vector<double, 3>> Spline::calc_spline() {

    std::vector<Vector<double,3>> spline;
    Vector<double, 3> res, tmp0,tmp1,tmp2,tmp3,tmp4,tmp5;

    while(this->get_current_timestamp() <=1)
    {
        double coeff0 = calc_num(this->current_timestamp,5);
        double coeff1 = 5*calc_num(this->current_timestamp,4) * this->current_timestamp;
        double coeff2 = 10*calc_num(this->current_timestamp,3) * (this->current_timestamp) * (this->current_timestamp);
        double coeff3 = 10*calc_num(this->current_timestamp,2)*this->current_timestamp * this->current_timestamp*this->current_timestamp;
        double coeff4 = 5*calc_num(this->current_timestamp,1)*this->current_timestamp * this->current_timestamp*this->current_timestamp*this->current_timestamp;
        double coeff5 = this->current_timestamp * this->current_timestamp*this->current_timestamp*this->current_timestamp*this->current_timestamp;

        tmp0 = coeff0*this->p0;
        tmp1 = coeff1*this->p1;
        tmp2 = coeff2*this->p2;
        tmp3 = coeff3*this->p3;
        tmp4 = coeff4*this->p4;
        tmp5 = coeff5*this->p5;

        res = tmp0 + tmp1 + tmp2 + tmp3 + tmp4 + tmp5;
        std::cout << this->get_current_timestamp() << std::endl;
        this->recalc_timestamp();

        spline.push_back(res);
    }

    return spline ;
}




// -----------How to handle----------- //
/*
Vector<double, 3> start;
start[0] = 5.5;
start[1] = 7.9;
start[2] = 4.3;


Vector<double, 3> end;
end[0] = 16.9;
end[1] = 12.9;
end[2] = 13.9;

Vector<double, 3> test;

start.output();
end.output();
test = (2.0*start)+end;
test.output();

Spline sp(0.1,0.1,0.1,0.1,0.1,start,end);
std::cout << sp.get_current_timestamp() << std::endl;

std::vector<Vector<double,3>> result;

result = sp.calc_spline();

for(Vector<double,3> &i :  result){
i.output();
}

*/
