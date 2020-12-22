//
// Created by Lukas on 22.12.2020.
//

#ifndef SDIR_CTRL2020_SINGLE_TRAJECTORY_H
#define SDIR_CTRL2020_SINGLE_TRAJECTORY_H

#include <Robot.h>

/**
 * This class acts as the base class for the different kinds of trajectories,
 * which can be used. Itself does not provide any functionality.
 */
class Single_trajectory
{

public:
  /**
   * Constructor - only stores the data
   *
   * @param joint   Joint index
   * @param qi      Initial joint angle
   * @param qf      Final joint angle
   */
  Single_trajectory(int joint, double qi, double qf);

  virtual ~Single_trajectory() = default;

  /**
   * Evaluates the calculated Trajectory at the given time step.
   *
   * @param t Time at which the trajectory should be evaluated
   * @return  Joint angle at the given time.
   */
  virtual double eval(double t) = 0;

  /**
   * @return The total duration of the trajectory
   */
  double get_duration();

protected:
  /**
   * Essential trajectory variables
   */
  double qi;  // Initial joint angle
  double qf;  // Final joint angle
  double tf;  // Final time / duration
  int joint;  // Joint index

  static Robot* robot;
};

#endif //SDIR_CTRL2020_SINGLE_TRAJECTORY_H
