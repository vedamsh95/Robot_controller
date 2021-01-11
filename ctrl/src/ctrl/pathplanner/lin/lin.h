#ifndef SDRI_CTRL2019_LIN_H
#define SDRI_CTRL2019_LIN_H

#include <Configuration.h>
#include <Trajectory.h>

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
     * @param velocity The velocity for the lin movement used in the corresponding trajectory
     * @param acceleration The acceleration for lin movement used in the corresponding trajectory
     * @return {@ref Trajectoy} for the movement of the robot
     */
    Trajectory* get_lin_trajectory(Configuration* _start_cfg, Configuration* _end_cfg, double velocity, double acceleration);
};


#endif //SDRI_CTRL2019_LIN_H
