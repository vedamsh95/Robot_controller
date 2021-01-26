#include <cmath>
#include <cfloat>
#include "ptp.h"

#define SECONDS_PER_STEP 0.05

const double MIN_JOINT_ANGLES[6] = {-185, -140, -120, -350, -125, -350};
const double MAX_JOINT_ANGLES[6] = {185, -5, 168, 350, 125, 350};
const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {10, 10, 10, 10, 10}; // in °/s²

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
    for (int i = 0; i < 6; i++) {
        if (start_cfg[i] < MIN_JOINT_ANGLES[i] * TO_RAD) start_cfg[i] = MIN_JOINT_ANGLES[i] * TO_RAD;
        if (start_cfg[i] > MAX_JOINT_ANGLES[i] * TO_RAD) start_cfg[i] = MAX_JOINT_ANGLES[i] * TO_RAD;
        if (end_cfg[i] < MIN_JOINT_ANGLES[i] * TO_RAD) end_cfg[i] = MIN_JOINT_ANGLES[i] * TO_RAD;
        if (end_cfg[i] > MAX_JOINT_ANGLES[i] * TO_RAD) end_cfg[i] = MAX_JOINT_ANGLES[i] * TO_RAD;
    }

    // Calculate time_final and distance for all joints
    double time_c[6]; // tc, ???
    double time_final[6]; // tfinal, duration of the movements for each joint
    double full_duration = -DBL_MAX; // maximum of time_final -> duration of the full movement
    for (int i = 0; i < 6; i++) {
        time_c[i] = MAX_VELOCITY[i] / MAX_ACCELERATION[i];
        time_final[i] = time_c[i] + ((end_cfg[i] - start_cfg[i]) / (MAX_VELOCITY[i] * TO_RAD));
        if (isinf(time_final[i])) time_final[i] = -1; // TODO: when does this happen?
        if (time_final[i] > full_duration) full_duration = time_final[i];
    }

    printf("DURATION: %f\n", full_duration);
    printf("START: %f %f %f %f %f %f\n", start_cfg[0], start_cfg[1], start_cfg[2], start_cfg[3], start_cfg[4], start_cfg[5]);

    // asynchronous max velocity
    for (double t = 0; t < full_duration; t += SECONDS_PER_STEP) {
        array<double, 6> new_cfg{};
        for (int i = 0; i < 6; i++) {
            // TODO: synchronous movements?
            if (0 <= t && t < time_c[i]) {
                new_cfg[i] = start_cfg[i] + 0.5 * MAX_ACCELERATION[i] * TO_RAD * pow(t, 2);
            } else if (t <= time_final[i] - time_c[i]) {
                new_cfg[i] = start_cfg[i] + 0.5 * MAX_ACCELERATION[i] * TO_RAD * time_c[i] * (t - time_c[i] / 2);
            } else if (t <= time_final[i]) {
                new_cfg[i] = end_cfg[i] - 0.5 * MAX_ACCELERATION[i] * pow(time_final[i] - t, 2);
            } else {
                new_cfg[i] = end_cfg[i];
            }
        }
        printf("%f %f %f %f %f %f\n", new_cfg[0], new_cfg[1], new_cfg[2], new_cfg[3], new_cfg[4], new_cfg[5]);
        config.push_back(new Configuration(new_cfg));
    }
    config.push_back(new Configuration(end_cfg));

    printf("END: %f %f %f %f %f %f\n", end_cfg[0], end_cfg[1], end_cfg[2], end_cfg[3], end_cfg[4], end_cfg[5]);

    trajectory->set_trajectory(config);

    return trajectory;
}

