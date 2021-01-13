#include "Trapezoidal_trajectory.h"

// More information about the formulas used can be found in the corresponding presentation:
// TRAJECTORIES AND VELOCITY PROFILES

Trapezoidal_trajectory::Trapezoidal_trajectory(double v, double a, double qi, double qf)
        : Single_trajectory(v, a, qi, qf) {
    tc = v / a;
    tf = tc + (abs((qf - qi)) / (v));
}

Trapezoidal_trajectory::Trapezoidal_trajectory(double v, double a, double qi, double qf, double tf)
        : Single_trajectory(v, a, qi, qf) {
    this->tf = tf;
    tc = 0.5 * (tf - sqrt(tf * tf - 4 * abs(qf - qi) / a));
}

template<typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double Trapezoidal_trajectory::eval(double t) {
    if (t < 0) {
        return qi;
    } else if (t < tc) {
        return qi + 0.5 * ac * t * t * sgn(qf - qi);
    } else if (t < tf - tc) {
        return qi + ac * tc * (t - tc / 2) * sgn(qf - qi);
    } else if (t < tf) {
        return qf - 0.5 * ac * (tf - t) * (tf - t) * sgn(qf - qi);
    } else {
        return qf;
    }
}
