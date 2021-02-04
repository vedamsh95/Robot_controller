#ifndef SDRI_CTRL2019_SPLINE_H
#define SDRI_CTRL2019_SPLINE_H


#include <Trajectory.h>
#include <SixDPos.h>

//define the Robot constants
const double d = 215; //215 mm  in-line wrist length
const double m = 330; //330 mm  offset length from Zo axis
const double n = 645;  //645 mm  offset height from base
const double o = 115;  // 115 mm  offset of arm on linkarm
const double a = 1150;  // 1150 mm  linkarm 
const double b = 1220;  //1220 mm    arm



class Spline {
public:

    Trajectory* get_spline_trajectoy(Configuration* _start_cfg, std::vector<SixDPos*> coordinates_list);
    void interpolate(double p0, double p5, vector<double>& path, double ts, double te, double as, double ae);
    std::tuple<double, double> Spline::get_velocity(Configuration *_start_cfg, Configuration *_end_cfg);
    double trap_prof(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin);
    double S_Functon(double M1[1][6], double M2[6][1]);
    void find_best_cfg(SixDPos P0, SixDPos P5, Configuration* cfg1, Configuration* cfg2);

};


#endif //SDRI_CTRL2019_PTP _H
