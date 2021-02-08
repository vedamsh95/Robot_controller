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
    void find_P1234(double P0, double P1, double P2, double P3, double P4, double P5, double d0, double ts, double as, double te, double ae);
    double Interpolate(double t, double P0, double P1, double P2, double P3, double P4, double P5);
    double compute_1(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin);

};


#endif //SDRI_CTRL2019_PTP _H
