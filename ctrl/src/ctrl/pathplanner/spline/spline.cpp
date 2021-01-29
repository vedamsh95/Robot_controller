#include <cmath>
#include <cfloat>
#include "spline.h"

#define SECONDS_PER_STEP 0.05
#define PTP_DEBUG true

const double MIN_JOINT_ANGLES[6] = {-185, -140, -120, -350, -125, -350};
const double MAX_JOINT_ANGLES[6] = {185, -5, 168, 350, 125, 350};
const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {300, 300, 300, 300, 300, 300}; // in °/s²

const double TO_RAD = M_PI / 180;

Trajectory* Spline::get_spline_trajectory(vector<SixDPos*> path)
{
    //computation of a ptp trajectory with the corresponding velocity profile
    Trajectory* trajectory = new Trajectory();
    vector<Configuration *> config;

    trajectory->set_trajectory(config);
    return trajectory;
}

