#include <cmath>
#include <cfloat>
#include "ptp.h"

#define SECONDS_PER_STEP 0.05

const double MIN_JOINT_ANGLES[6] = {-185, -140, -120, -350, -125, -350};
const double MAX_JOINT_ANGLES[6] = {185, -5, 168, 350, 125, 350};
const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {100, 100, 100, 100, 100}; // in °/s²

const double TO_RAD = M_PI / 180;

Trajectory* Ptp::get_ptp_trajectory(Configuration* _start_cfg, Configuration* _end_cfg)
{
    //computation of a ptp trajectory with the corresponding velocity profile
    Trajectory* trajectory = new Trajectory();
    //Dummy trajectory
    vector<Configuration *> config;
    array<double, 6> start_cfg = _start_cfg->get_configuration();
    array<double, 6> end_cfg = _end_cfg->get_configuration();

    // TODO: check MIN_JOINT_ANGLES and MAX_JOINT_ANGLES

    // Calculate time_final and distance for all joints
    double time_final[6]; // tf, duration of the movements for each joint
    double distance[6]; // D, distance of the joint to each other
    double full_duration = -DBL_MAX; // maximum of time_final -> duration of the full movement
    for (int i = 0; i < 6; i++) {
        if (start_cfg[i])
        distance[i] = (end_cfg[i] - start_cfg[i]);
        time_final[i] = sqrt(2 * distance[i] / (MAX_ACCELERATION[i] * TO_RAD));
        if (time_final[i] > full_duration) full_duration = time_final[i];
    }

    // asynchronous max velocity
    for (double t = 0; t < full_duration; t += SECONDS_PER_STEP) {
        array<double, 6> new_cfg{};
        for (int i = 0; i < 6; i++) {
            // TODO: synchronous movements?
            if (0 <= t < time_final[i] / 2) {
                new_cfg[i] = start_cfg[i] + 2 * (t / time_final[i]) * distance[i];
            } else if (t <= time_final[i]) {
                new_cfg[i] = start_cfg[i] + (-1 + 4 * (t / time_final[i]) - 2 * pow(t / time_final[i], 2)) * distance[i];
            }
        }
        config.push_back(new Configuration(new_cfg));
    }

    trajectory->set_trajectory(config);

    return trajectory;
}

