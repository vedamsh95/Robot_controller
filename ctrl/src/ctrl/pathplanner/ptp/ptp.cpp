#include <cmath>
#include <cfloat>
#include "ptp.h"

#define SECONDS_PER_STEP 0.05
#define PTP_DEBUG true

const double MIN_JOINT_ANGLES[6] = {-185, -140, -120, -350, -125, -350};
const double MAX_JOINT_ANGLES[6] = {185, -5, 168, 350, 125, 350};
const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {300, 300, 300, 300, 300, 300}; // in °/s²

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
    double distance[6];
    double time_c[6]; // tc, duration of the acceleration/deceleration
    double time_final[6]; // tfinal, duration of the movements for each joint
    double time_maxvelocity[6];
    double full_duration = -DBL_MAX; // maximum of time_final -> duration of the full movement
    for (int i = 0; i < 6; i++) {
        distance[i] = end_cfg[i] - start_cfg[i];
        time_c[i] = (MAX_VELOCITY[i] * TO_RAD) / (MAX_ACCELERATION[i] * TO_RAD);
        time_final[i] = time_c[i] + (abs(distance[i]) / (MAX_VELOCITY[i] * TO_RAD));
        time_maxvelocity[i] = sqrt(2.0 * abs(distance[i]) / (MAX_ACCELERATION[i] * TO_RAD));
        if (time_c[i] * 2 >= time_final[i] && time_maxvelocity[i] > full_duration) full_duration = time_maxvelocity[i];
        if (!(time_c[i] * 2 >= time_final[i]) && time_final[i] > full_duration) full_duration = time_final[i];
    }

    if (PTP_DEBUG) {
        for (int i = 0; i < 6; i++) {
            if (time_c[i] * 2 >= time_final[i]) {
                printf("Axis %d will take %f seconds using a maximum velocity profile\n", i, time_maxvelocity[i]);
            } else {
                printf("Axis %d will take %f seconds using a trapezoidal profile (%f seconds for acceleration/deceleration)\n", i, time_final[i], time_c[i]);
            }
        }
        printf("START: %f %f %f %f %f %f\n", start_cfg[0], start_cfg[1], start_cfg[2], start_cfg[3], start_cfg[4],
               start_cfg[5]);
    }

    // asynchronous trapezoidal profile
    for (double t = 0; t < full_duration; t += SECONDS_PER_STEP) {
        array<double, 6> new_cfg{};
        for (int i = 0; i < 6; i++) {
            // TODO: synchronous movements?
            if (time_c[i] * 2 >= time_final[i]) {
                // we need a max velocity profile (the acceleration time is too short)
                if (t < time_maxvelocity[i] / 2.0) {
                    if (PTP_DEBUG) printf("       Axis %d: max-velocity, accelerating\n", i+1);
                    new_cfg[i] = start_cfg[i] + 2.0 * pow(t / time_maxvelocity[i], 2.0) * distance[i];
                } else if (t < time_maxvelocity[i]) {
                    if (PTP_DEBUG) printf("       Axis %d: max-velocity, decelerating\n", i+1);
                    new_cfg[i] = start_cfg[i] +
                                 (-1.0 + 4.0 * (t / time_maxvelocity[i]) - 2.0 * pow(t / time_maxvelocity[i], 2.0)) * distance[i];
                } else {
                    new_cfg[i] = end_cfg[i];
                }
            } else {
                // we need a trapezoidal profile
                int sign = start_cfg[i] < end_cfg[i] ? 1 : -1;
                if (t < time_c[i]) {
                    if (PTP_DEBUG) printf("       Axis %d: trapezoidal, accelerating\n", i+1);
                    new_cfg[i] = start_cfg[i] + sign * 0.5 * (MAX_ACCELERATION[i] * TO_RAD) * pow(t, 2);
                } else if (t <= time_final[i] - time_c[i]) {
                    if (PTP_DEBUG) printf("       Axis %d: trapezoidal, reached maximum velocity\n", i+1);
                    new_cfg[i] =
                            start_cfg[i] + sign * (MAX_ACCELERATION[i] * TO_RAD) * time_c[i] * (t - (time_c[i] / 2));
                } else if (t <= time_final[i]) {
                    if (PTP_DEBUG) printf("       Axis %d: trapezoidal, decelerating\n", i+1);
                    new_cfg[i] = end_cfg[i] - sign * 0.5 * (MAX_ACCELERATION[i] * TO_RAD) * pow(time_final[i] - t, 2);
                } else {
                    new_cfg[i] = end_cfg[i];
                }
            }
        }
        if (PTP_DEBUG) printf(" NEXT: %f %f %f %f %f %f\n", new_cfg[0], new_cfg[1], new_cfg[2], new_cfg[3], new_cfg[4], new_cfg[5]);
        config.push_back(new Configuration(new_cfg));
    }
    config.push_back(new Configuration(end_cfg));

    if (PTP_DEBUG) printf("  END: %f %f %f %f %f %f\n", end_cfg[0], end_cfg[1], end_cfg[2], end_cfg[3], end_cfg[4], end_cfg[5]);

    trajectory->set_trajectory(config);

    return trajectory;
}

