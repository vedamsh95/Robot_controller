#ifndef SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H
#define SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H

#include <cmath>
#include "Single_trajectory.h"

/**
 * This class implements a trapezoidal trajectory. It can only be used
 * if the given joint difference is too large for a maximum velocity
 * trajectory. This can be checked using the static method select_type
 * from the Single_trajectory class. It consists of an acceleration
 * phase, a constant velocity phase and a deceleration phase.
 */
class Trapezoidal_trajectory : public Single_trajectory {

public:
    /**
     * Constructor - Calculates the fastest trajectory
     *
     * @param v       Final velocity
     * @param a       Used acceleration
     * @param qi      Initial joint angle
     * @param qf      Final joint angle
     */
    Trapezoidal_trajectory(double v, double a, double qi, double qf);

    /**
     * Constructor - Calculates a trajectory with the given duration
     *               The duration must be larger than the smallest possible
     *               duration.
     *
     * @param v       Final velocity
     * @param a       Used acceleration
     * @param qi      Initial joint angle
     * @param qf      Final joint angle
     * @param tf      Desired final time
     */
    Trapezoidal_trajectory(double v, double a, double qi, double qf, double tf);

    /**
     * Evaluates the calculated Trajectory at the given time step.
     *
     * @param t Time at which the trajectory should be evaluated
     * @return  Joint angle at the given time.
     */
    double eval(double t) override;

private:

    double tc;  // Time after which the constant velocity phase begins

};


#endif //SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H
