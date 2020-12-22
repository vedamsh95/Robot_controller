#include "Trapezoidal_trajectory.h"

Trapezoidal_trajectory::Trapezoidal_trajectory(int joint, double qi, double qf)
    : Single_trajectory(joint, qi, qf)
{
  double a_max = robot->accelerations[joint];
  double v_max = robot->velocities[joint];
  tc = v_max / a_max;
  tf = tc + (abs((qf - qi)) / (v_max));
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

double Trapezoidal_trajectory::eval( double t )
{
  double a_max = robot->accelerations[joint];

  if ( t < 0) {
    return qi;
  } else if (t < tc) {
    return qi + 0.5 * a_max * t * t * sgn(qf-qi);
  } else if ( t < tf - tc) {
    return qi + a_max * tc * (t-tc/2)* sgn(qf-qi);
  } else if ( t < tf ) {
    return qf - 0.5 * a_max * (tf - t) * (tf- t) * sgn(qf-qi);
  } else {
    return qf;
  }
}
