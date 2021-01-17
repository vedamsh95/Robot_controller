//
// Created by Timo on 07.01.2021.
//

#ifndef SDIR_CTRL2020_SPLINE_H
#define SDIR_CTRL2020_SPLINE_H


#include "Vector.h"
#include <vector>
#include <json.h>
#include "Trajectory.h"


class Spline {

private:
    Vector<double,3> start_position, start_orientation;
    double speed, acceleration, num_points;
    std::vector<Vector<double, 3>> *points;

    double current_timestamp, a_s, a_e,t_s, t_e,t;

public:
    Spline(Vector<double,3> start_point, Vector<double,3> start_orientation,std::vector<Vector<double, 3>> *points, double speed, double acceleration);
    void out();
    Trajectory* calculateSpline();


    Spline(double _t, double _ts, double _te, double _as, double _ae, Vector<double,3> start, Vector<double,3> end);
    double get_current_timestamp() const;
    void recalc_timestamp();
    double  calc_num(double num, int n);
    std::vector<Vector<double,3>> calc_spline();

};


#endif //SDIR_CTRL2020_SPLINE_H
