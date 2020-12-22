#ifndef SDIR_CTRL2020_MAX_VEL_TRAJECTORY_H
#define SDIR_CTRL2020_MAX_VEL_TRAJECTORY_H

#include <cmath>
#include "Single_trajectory.h"

/**
 * This class implements a maximum velocity trajectory. It can only be used
 * if the given joint difference can be achieved using this kind of trajectory.
 * It consists of an acceleration phase and a deceleration phase. It makes use
 * of the Robot.h file which contains the robots technical specification.
 */
class Max_vel_trajectory : public Single_trajectory
{

public:
  /**
   * Constructor - calculates the necessary values that describe the trajectory
   *
   * @param joint   Joint index
   * @param qi      Initial joint angle
   * @param qf      Final joint angle
   */
  Max_vel_trajectory(int joint, double qi, double qf);

  /**
   * Evaluates the calculated Trajectory at the given time step.
   *
   * @param t Time at which the trajectory should be evaluated
   * @return  Joint angle at the given time.
   */
  double eval(double t) override;

private:

  double qm;  // Joint angle in the middle of the motion

};


#endif //SDIR_CTRL2020_MAX_VEL_TRAJECTORY_H
