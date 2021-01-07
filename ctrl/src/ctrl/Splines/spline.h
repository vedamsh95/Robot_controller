//
// Created by Timo on 07.01.2021.
//

#ifndef SDIR_CTRL2020_SPLINE_H
#define SDIR_CTRL2020_SPLINE_H


#include "Vector.h"
#include <vector>


class Spline {

private:
    Vector<double,3> p0,p1,p2,p3,p4, p5;
    double current_timestamp, a_s, a_e,t_s, t_e,t;

public:
    Spline(double _t, double _ts, double _te, double _as, double _ae, Vector<double,3> start, Vector<double,3> end);
    double get_current_timestamp() const;
    void recalc_timestamp();
    double  calc_num(double num, int n);
    std::vector<Vector<double,3>> calc_spline();

};


#endif //SDIR_CTRL2020_SPLINE_H
