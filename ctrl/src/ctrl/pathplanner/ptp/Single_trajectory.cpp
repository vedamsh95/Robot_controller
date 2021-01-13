#include "Single_trajectory.h"

Robot *Single_trajectory::robot = &Robot::getInstance();

Single_trajectory::Single_trajectory(double v, double a, double qi, double qf)
        : ve(v), ac(a), qi(qi), qf(qf), tf(0) {}

double Single_trajectory::get_duration() const {
    return tf;
}

Single_trajectory::Type Single_trajectory::select_type(double distance, double maxVel, double maxAcc) {
    // Maximum distance that can be covered with a maximum
    // velocity profile given the maximal velocity
    double comp = maxVel * maxVel / maxAcc;

    if (distance - comp < 0) {
        return Type::MAX_VEL;
    } else {
        return Type::TRAPEZOIDAL;
    }
}
