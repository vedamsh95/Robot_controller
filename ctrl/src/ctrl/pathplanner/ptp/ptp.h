#ifndef SDRI_CTRL2019_PTP_H
#define SDRI_CTRL2019_PTP_H


#include <Trajectory.h>
#include <Robot.h>
#include <functional>
#include <math.h>
#include <iostream>
#include <assert.h>
#include "Single_trajectory.h"
#include "Max_vel_trajectory.h"
#include "Trapezoidal_trajectory.h"
#include "../../matplotlib-cpp-master/matplotlibcpp.h"

/**
 * This class implements the ptp movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * Usage examples that might be helpful for the lin movement (and maybe spline):
 * Given the points A and B, as well as a velocity and an acceleration: Create a lin movement from A to B
 * ##### NO COMPLETE C++ CODE, CONTAINS PSEUDO CODE #####
 *
 * distance = length(B-A)
 * trajectory_type = Single_trajectory::selectType(distance, velocity, acceleration)
 * Single_trajectory* trajectory;
 *
 * if (type == Single_trajectory::Type::MAX_VEL) {
 *   trajectory = new Max_vel_trajectory(velocity, acceleration, 0, distance);
 * } else if (type == Single_trajectory::Type::TRAPEZOIDAL) {
 *   trajectory = new Trapezoidal_trajectory(velocity, acceleration, 0, distance);
 * }
 *
 * auto tmp = static_cast<float>(trajectory->get_duration() / Robot::getInstance().time_interval);
 * size_t cycles = roundf(tmp) + 3;
 *
 * vector<Point> points;
 * double t = 0;
 * for (size_t c = 0; c < cycles; c++) {
 *   value = single_trajectories[i]->eval(t);
 *   factor = value / distance;
 *   point = A + factor * (B-A)
 *   points.push_back(point);
 *   t += Robot::getInstance().time_interval;
 * }
 *
 */
class Ptp {

private:

  Robot* robot;
  Trajectory* trajectory;

public:

  /**
   * Default constructor
   */
  Ptp();

  /**
   * Example function computing the trajectory for a given path (defined by two configurations) as ptp movement.
   *
   * @param _start_cfg {@ref Configuration} of the starting point of the path
   * @param _end_cfg {@ref Configuration} of the target point of the path
   * @param sync Whether the motion should be synchronous
   * @return {@ref Trajectory} for the movement of the robot
   */
  Trajectory* get_ptp_trajectory(Configuration* _start_cfg, Configuration* _end_cfg, bool sync);

  /**
   * Checks for a given configuration whether it is feasible and if not,
   * it changes the values to the limits of the robot.
   *
   * @param cfg Configuration to check and change
   */
  void makeFeasible(Configuration* cfg);

  /**
   * Plots the given configuration using the matplotlibcpp library to visualise the trajectories
   * @param configs     The configurations of the ptp movement to plot
   */
  static void plotMovement(vector<Configuration*> &configs);

};


#endif //SDRI_CTRL2019_PTP_H
