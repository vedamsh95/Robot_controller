#ifndef SDRI_CTRL2019_LIN_H
#define SDRI_CTRL2019_LIN_H

#include <Configuration.h>
#include <Trajectory.h>
#include <fw_kinematics.h>
#include <sdir_ctrl.h>
#include "inverse_kinematics.h"
#include "Vector.h"

/**
 * This class implementes the lin movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., acceleration, velocity, and rotation
 *       values of the physical joints.
 */
class Lin {
public:
    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as lin movement.
     *
     * @param _start_cfg {@ref Configruation} of the starting point of the path
     * @param _end_cfg {@ref Configruation} of the target point of the path
     * @return {@ref Trajectoy} for the movement of the robot
     */
    Trajectory* get_lin_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg, double speed, double acceleration);
    Vector<double, 3> trapezoidal_prof(Vector<double, 3> start_pos, Vector<double, 3> end_pos,
                                       double t_f, double t, double t_c, double a_max, double v_max, Vector<double, 3> path_dir_vec);
    Vector<double, 3> max_vel_profile(Vector<double, 3> start_pos,double distance, double t_f, double t_m, double t, Vector<double, 3> path_dir_vec );



};


#endif //SDRI_CTRL2019_LIN_H
