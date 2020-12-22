#include "Single_trajectory.h"

Robot* Single_trajectory::robot = &Robot::getInstance();

Single_trajectory::Single_trajectory(int joint, double qi, double qf)
    : joint(joint), qi(qi), qf(qf), tf(0) {}

double Single_trajectory::get_duration()
{
  return tf;
}
