#ifndef SDIR_CTRL2020_ROBOT_H
#define SDIR_CTRL2020_ROBOT_H

#define _USE_MATH_DEFINES
#include <math.h>

/**
 * This is a singleton class that provides access to the the technical data
 * of the robot.
 */
class Robot
{
public:
  /**
   * The only way of accessing this class, e.g.
   * Robot::getInstance().velocities[0]
   * or
   * auto robot = &Robot::getInstance();
   * robot->limits[1].min;
   *
   * @return Instance of this class
   */
  static Robot& getInstance()
  {
    static Robot instance;
    return instance;
  }

  /**
   * Prevent duplication by the copy constructor or initialization
   */
  Robot(Robot const&) = delete;
  void operator=(Robot const&) = delete;

  /**
   * Structure that holds a range
   */
  struct Range {
    double min;
    double max;
  };

  /**
   * Motion range for the 6 axes given in [radiant] and in the form of a Range
   */
  const Range limits[6] = {
    {-185 / 180.0 * M_PI, +185 / 180.0 * M_PI},
    {-140 / 180.0 * M_PI, -  5 / 180.0 * M_PI},
    {-120 / 180.0 * M_PI, +168 / 180.0 * M_PI},
    {-350 / 180.0 * M_PI, +350 / 180.0 * M_PI},
    {-125 / 180.0 * M_PI, +125 / 180.0 * M_PI},
    {-350 / 180.0 * M_PI, +350 / 180.0 * M_PI},
  };

  /**
   * Maximum velocities for the six axes in [radiant/s]
   */
  const double velocities[6] = {
          120 / 180.0 * M_PI,
          115 / 180.0 * M_PI,
          120 / 180.0 * M_PI,
          190 / 180.0 * M_PI,
          180 / 180.0 * M_PI,
          260 / 180.0 * M_PI,
  };

  /**
   * Maximum acceleration for the six axes (Just some values, not the correct ones!!!)
   */
   const double accelerations[6] = {
           360 / 180.0 * M_PI,
           360 / 180.0 * M_PI,
           360 / 180.0 * M_PI,
           360 / 180.0 * M_PI,
           360 / 180.0 * M_PI,
           360 / 180.0 * M_PI
   };

private:
  /**
   * Private constructor to prevent initialization
   */
  Robot() = default;
};


#endif //SDIR_CTRL2020_ROBOT_H
