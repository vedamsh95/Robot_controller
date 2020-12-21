#ifndef SDRI_CTRL2019_PTP_H
#define SDRI_CTRL2019_PTP_H


#include <Trajectory.h>
#include <Robot.h>
#include <functional>
#include <math.h>
#include <iostream>

/**
 * This class implements the ptp movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., acceleration, velocity, and rotation
 *       values of the physical joints.
 */
class Ptp {

private:

  static Robot* robot;

  /**
   * Classes that represents an individual trajectory for a single joint.
   */
  class Single_trajectory {
  protected:
    double qi;
    double qf;
    double tf;
    int joint;
  public:
    virtual double eval(double t) = 0;
    double get_duration();
    Single_trajectory(int joint, double qi, double qf);
  };

  class Max_vel_trajectory : public Single_trajectory {
  private:
    double qm;
  public:
    Max_vel_trajectory(int joint, double qi, double qf);
    double eval(double t);
  };

  class Trapezoidal_trajectory : public Single_trajectory {
  private:
    double tc;
  public:
    Trapezoidal_trajectory(int joint, double qi, double qf);
    double eval(double t);
  };

  /**
   * End of the classes that represents an individual trajectory for a single joint
   */

public:

  /**
   * Example function computing the trajectory for a given path (defined by two configurations) as ptp movement.
   *
   * @param _start_cfg {@ref Configuration} of the starting point of the path
   * @param _end_cfg {@ref Configuration} of the target point of the path
   * @return {@ref Trajectory} for the movement of the robot
   */
  Trajectory* get_ptp_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg);

  bool isFeasible(Configuration* cfg);

};


#endif //SDRI_CTRL2019_PTP_H
