#ifndef SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H
#define SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H

#include <cmath>
#include "Single_trajectory.h"

/**
 * This class implements a trapezoidal trajectory. It can only be used
 * if the given joint difference is too large for a maximum velocity
 * trajectory. It consists of an acceleration phase, a constant velocity
 * phase and a deceleration phase. It makes use of the Robot.h file which
 * contains the robots technical specification.
 */
class Trapezoidal_trajectory : public Single_trajectory {

public:
  /**
   * Constructor - calculates the necessary values that describe the trajectory
   *
   * @param joint   Joint index
   * @param qi      Initial joint angle
   * @param qf      Final joint angle
   */
  Trapezoidal_trajectory(int joint, double qi, double qf);

  /**
   * Evaluates the calculated Trajectory at the given time step.
   *
   * @param t Time at which the trajectory should be evaluated
   * @return  Joint angle at the given time.
   */
  double eval(double t);

private:

  double tc;  // Time after which the constant velocity phase begins

};


#endif //SDIR_CTRL2020_TRAPEZOIDAL_TRAJECTORY_H
