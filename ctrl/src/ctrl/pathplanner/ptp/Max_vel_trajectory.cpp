#include "Max_vel_trajectory.h"

// More information about the formulas used can be found in the corresponding presentation:
// TRAJECTORIES AND VELOCITY PROFILES

Max_vel_trajectory::Max_vel_trajectory(double v, double a, double qi, double qf)
        : Single_trajectory(v, a, qi, qf) {
    qm = (qf - qi) * 0.5;
    tf = 2 * sqrt((2 * abs(qm) / a));
}

double Max_vel_trajectory::eval(double t) {
    if (t < 0) {
        return qi;
    } else if (t < tf / 2) {
        return qi + 2 * pow(t / tf, 2) * (qf - qi);
    } else if (t < tf) {
        return qi + (-1 + 4 * (t / tf) - 2 * pow(t / tf, 2)) * (qf - qi);
    } else {
        return qf;
    }
}