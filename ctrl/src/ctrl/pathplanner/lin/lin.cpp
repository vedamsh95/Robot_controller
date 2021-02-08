#include <cmath>
#include <cfloat>
#include <iostream>

#include "lin.h"
#include "../../kinematics/direct/fw_kinematics.h"
#include "../../kinematics/inverse/inverse_kinematics.h"

#define SECONDS_PER_STEP 0.05
const double MIN_JOINT_ANGLES[6] = {-185, -140, -120, -350, -125, -350};
const double MAX_JOINT_ANGLES[6] = {185, -5, 168, 350, 125, 350};
const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {10, 10, 10, 10, 10, 10}; // in °/s²
const double TO_RAD = M_PI / 180;

Trajectory *Lin::get_lin_trajectory(Configuration *_start_cfg, Configuration *_end_cfg) {
    //TODO: IMPLEMENT! implement the computation of a lin trajectory with the corresponding velocity profile
    Trajectory *traj = new Trajectory();
    vector<Configuration *> config{_start_cfg};

    //Call FwKinematics
    FwKinematics fwKinematics;
    InvKinematics invKinematics;
    SixDPos *start_pos = fwKinematics.get_fw_kinematics(_start_cfg);
    SixDPos *end_pos = fwKinematics.get_fw_kinematics(_end_cfg);

    vector<double> distance = {
            end_pos->get_X() - start_pos->get_X(),
            end_pos->get_Y() - start_pos->get_Y(),
            end_pos->get_Z() - start_pos->get_Z(),
            end_pos->get_A() - start_pos->get_A(),
            end_pos->get_B() - start_pos->get_B(),
            end_pos->get_C() - start_pos->get_C(),
    };
    //double progress = 0;
    SixDPos *new_pos = new SixDPos();

    // Version using a trapezoidal profile along the path
    double t = 0;
    do {
        vector<double> *new_p = new vector<double>();
        t += SECONDS_PER_STEP;
        for (int j = 0; j < 6; ++j) {
            // calculate next point

            double t_c;
            t_c = MAX_VELOCITY[j] / MAX_ACCELERATION[j];
            double t_final;
            t_final = t_c + ((*end_pos)[j] - (*start_pos)[j]) / MAX_VELOCITY[j];

            if (t >= 0 && t < t_c) { // end position not reached yet
                //new_pos[j] = new SixDPos(start_pos[j] + 0.5 * MAX_ACCELERATION[j] * t[j] * t[j]);
                /*new_pos[j]*/
                new_p->push_back((*start_pos)[j] + 0.5 * MAX_ACCELERATION[j] * t * t);
            } else if (t > t_c && t < (t_final - t_c)) {
                new_p->push_back((*start_pos)[j] + MAX_ACCELERATION[j] * t_c * (t - t_final / 2));
            } else if (t > (t_final - t_c) && t <= t_final) {
                new_p->push_back((*start_pos)[j] - 0.5 * MAX_ACCELERATION[j] * pow(t_final - t, 2));
            } else {
                // We're there!
            }
        }
        new_pos->set_position(*new_p);

        vector<Configuration *> *new_cfg_possibilities = invKinematics.get_inv_kinematics(new_pos);
        // TODO: get new configuration that's closest to "traj->get_last_configuration()"
        Configuration *bestConfiguration = nullptr;
        double bestDistance = DBL_MAX;
        for (int i = 0; i < new_cfg_possibilities->size(); i++) {
            double distance = 0;
            for (int j = 0; j < 6; j++) {
                distance += abs((*(*new_cfg_possibilities)[i])[j] - (*config[config.size() - 1])[j]) * MAX_VELOCITY[j];
            }
            if (distance < bestDistance) {
                bestDistance = distance;
                bestConfiguration = (*new_cfg_possibilities)[i];
            }
        }
        config.push_back(bestConfiguration);
    } while (
            abs(new_pos->get_X() - start_pos->get_X()) < abs(end_pos->get_X() - start_pos->get_X()) ||
            abs(new_pos->get_Y() - start_pos->get_Y()) < abs(end_pos->get_Y() - start_pos->get_Y()) ||
            abs(new_pos->get_Z() - start_pos->get_Z()) < abs(end_pos->get_Z() - start_pos->get_Z()) ||
            abs(new_pos->get_A() - start_pos->get_A()) < abs(end_pos->get_A() - start_pos->get_A()) ||
            abs(new_pos->get_B() - start_pos->get_B()) < abs(end_pos->get_B() - start_pos->get_B()) ||
            abs(new_pos->get_C() - start_pos->get_C()) < abs(end_pos->get_C() - start_pos->get_C())
            );

    traj->set_trajectory(config);
    return traj;
}