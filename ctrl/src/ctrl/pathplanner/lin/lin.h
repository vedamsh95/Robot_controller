#ifndef SDRI_CTRL2019_LIN_H
#define SDRI_CTRL2019_LIN_H

#include <Configuration.h>
#include <Trajectory.h>
#include <fw_kinematics.h>
#include <sdir_ctrl.h>
#include "inverse_kinematics.h"
#include "Vector.h"
#include "Singularities/Singularity.h"
#include <array>

/**
 * This class implementes the lin movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., acceleration, velocity, and rotation
 *       values of the physical joints.
 */
class Lin {
private:

    //-----------------------------------------------Special Case for Movement------------------------------------------

    int movement_case = -1;

    //-----------------------------------------------Vectors------------------------------------------------------------
    std::vector<Configuration*> config_vec;

    double config_switch_limit = ConfigProvider::getInstance().getconfig_switch_limit();
    int num_iterations = 0;
    int config_switch_indent = -1;

public:
    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as lin movement.
     *
     * @param _start_cfg {@ref Configruation} of the starting point of the path
     * @param _end_cfg {@ref Configruation} of the target point of the path
     * @return {@ref Trajectoy} for the movement of the robot
     */
    double M_PII = 3.141592654;
    //-------------------create vector to store acceleration and speed of every joint at every timepoint----------------
    std::vector<vector<double>> joint_velocities_vec;

    Trajectory* get_lin_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg, double speed, double acceleration);

    std::array<double,3> trapezoidal_prof(Vector<double, 3> start_pos, Vector<double, 3> end_pos,
                                       double t_f, double t, double t_c, double a_max, double v_max, Vector<double, 3> path_dir_vec);

    std::array<double,3> max_vel_profile(Vector<double, 3> start_pos,double distance, double t_f, double t_m, double t, Vector<double, 3> path_dir_vec );

    double calc_config_difference(Configuration* config1, Configuration* config2);

    bool checkconfiglimits(Configuration* config1, Configuration* config2,
                           std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps, bool add_v_to_vec);

//    void add_middle_cfg2(Configuration* config1, Configuration* config2,
//                              std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps);

    int get_best_config(std::vector<Configuration*>* temp_configs, double a_max, Vector<double, 6> joint_max_vel, double timesteps, bool checklimits);

    int add_middle_cfg(Configuration* config1, Configuration* config2,
                            std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps, int num_iterations2);

    std::vector<double> calc_config_difference_to_vec(Configuration* config1, Configuration* config2);

};


#endif //SDRI_CTRL2019_LIN_H
