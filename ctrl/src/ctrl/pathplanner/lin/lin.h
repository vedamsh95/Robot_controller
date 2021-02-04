#ifndef SDRI_CTRL2019_LIN_H
#define SDRI_CTRL2019_LIN_H

#include <Configuration.h>
#include <Trajectory.h>
#include <../kinematics/inverse/inverse_kinematics.h>
#include<../kinematics/direct/fw_kinematics.h>




/**
 * This class implementes the lin movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., accelaration, verlocity, and rotation
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
    Trajectory* get_lin_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg);
    double compute(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin);
    double new_speed(double in_angle, double fin_angle,double t_max);
    //InvKinematics* Lin::inv_solution(InvKinematics* configurations, Configuration* _start_cfg, double in_phi1, double in_phi2, double in_phi3, double in_phi4, double in_phi5, double in_phi6);
    InvKinematics* Lin::inv_solution(InvKinematics* solution, Configuration* _start_cfg, double in_phi1, double in_phi2, double in_phi3, double in_phi4, double in_phi5, double in_phi6, vector<Configuration*> configurations);
};




#endif //SDRI_CTRL2019_LIN_H