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

Trajectory* Lin::get_lin_trajectory(Configuration* _start_cfg, Configuration* _end_cfg)
{
    //TODO: IMPLEMENT! implement the computation of a lin trajectory with the corresponding velocity profile
    Trajectory* traj = new Trajectory();
    //Call FwKinematics
    FwKinematics fwKinematics;
    InvKinematics invKinematics;
    SixDPos* start_pos = fwKinematics.get_fw_kinematics(_start_cfg);
    SixDPos* end_pos = fwKinematics.get_fw_kinematics(_end_cfg);

    vector<double> distance = {
            end_pos->get_X() - start_pos->get_X(),
            end_pos->get_Y() - start_pos->get_Y(),
            end_pos->get_Z() - start_pos->get_Z(),
            end_pos->get_A() - start_pos->get_A(),
            end_pos->get_B() - start_pos->get_B(),
            end_pos->get_C() - start_pos->get_C(),
    };
    //double progress = 0;
    SixDPos* new_pos = new SixDPos();

    /*while (progress < 1) {

        Configuration *new_cfg;
        bool is_okay = false;
        double step = 0.1;
        //while (progress + step >= 1.01) {
        //    step = step / 2;
        //}
        while (!is_okay) {
            new_pos->set_position({
                                          start_pos->get_X() + distance[0] * (progress + step),
                                          start_pos->get_Y() + distance[1] * (progress + step),
                                          start_pos->get_Z() + distance[2] * (progress + step),
                                          start_pos->get_A() + distance[3] * (progress + step),
                                          start_pos->get_B() + distance[4] * (progress + step),
                                          start_pos->get_C() + distance[5] * (progress + step),
                                  });

            vector<Configuration *> *new_cfg_possibilities = invKinematics.get_inv_kinematics(new_pos);
            // TODO: get new configuration that's closest to "traj->get_last_configuration()"
            new_cfg = ???;
            // TODO: check that going from "traj->get_last_configuration()" to "new_cfg" in
            //       SECONDS_PER_STEP doesn't exceed MAX_VELOCITY for any joint
            // TODO: check that going from "traj->get_last_configuration()" to "new_cfg" in
            //       SECONDS_PER_STEP doesn't exceed MAX_ACCELERATION for any joint.
            //       The velocity of the last step is required, a big problem would be to slow down before the end.
            is_okay = ???;
            if (!is_okay) {
                step = step * 0.5;
            }
        }

        progress += step;
        traj->add_configuration(new_cfg);
    }

    return traj;*/

    // Version using a trapezoidal profile along the path
    double t = 0;
    do {
        vector<double> *new_p = new vector<double>();
        t += SECONDS_PER_STEP;
        for (int i; i < 6; ++i) {
            // calculate next point

            double t_c[i];
            t_c[i] = MAX_VELOCITY[i] / MAX_ACCELERATION[i];
            double t_final[i];
            t_final[i] = t_c[i] + ((*end_pos)[i] - (*start_pos)[i]) / MAX_VELOCITY[i];

            if (t >= 0 && t < t_c[i]) { // end position not reached yet
                //new_pos[i] = new SixDPos(start_pos[i] + 0.5 * MAX_ACCELERATION[i] * t[i] * t[i]);
                /*new_pos[i]*/
                new_p->push_back((*start_pos)[i] + 0.5 * MAX_ACCELERATION[i] * t * t);
            } else if (t > t_c[i] && t < (t_final[i] - t_c[i])) {
                new_p->push_back((*start_pos)[i] + MAX_ACCELERATION[i] * t_c[i] * (t - t_final[i] / 2));
            } else if (t > (t_final[i] - t_c[i]) && t <= t_final[i]) {
                new_p->push_back((*start_pos)[i] - 0.5 * MAX_ACCELERATION[i] * pow(t_final[i] - t, 2));
            } else {
                // We're there!
            }
        }
        new_pos->set_position(*new_p);
        vector<Configuration *> *new_cfg_possibilities = invKinematics.get_inv_kinematics(new_pos);
        // TODO: get new configuration that's closest to "traj->get_last_configuration()"
        Configuration* new_cfg = (*new_cfg_possibilities)[0];
        for (int i = 1; i < new_cfg_possibilities->size(); i++) {
            // is (*new_cfg_possibilities)[i] better than new_cfg?!
        }
        traj->add_configuration(new_cfg);
    } while (
            abs(new_pos->get_X() - start_pos->get_X()) < abs(end_pos->get_X() - start_pos->get_X()) ||
                    abs(new_pos->get_Y() - start_pos->get_Y()) < abs(end_pos->get_Y() - start_pos->get_Y()) ||
                    abs(new_pos->get_Z() - start_pos->get_Z()) < abs(end_pos->get_Z() - start_pos->get_Z()) ||
                    abs(new_pos->get_A() - start_pos->get_A()) < abs(end_pos->get_A() - start_pos->get_A()) ||
                    abs(new_pos->get_B() - start_pos->get_B()) < abs(end_pos->get_B() - start_pos->get_B()) ||
                    abs(new_pos->get_C() - start_pos->get_C()) < abs(end_pos->get_C() - start_pos->get_C())
            );

    return traj;
}