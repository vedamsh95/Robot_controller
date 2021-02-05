#ifndef SDRI_CTRL2019_SPLINE_H
#define SDRI_CTRL2019_SPLINE_H

#include <Configuration.h>
#include <Trajectory.h>

/**
 * This class implementes the spline movement from to given {@ref Configuration},
 * 
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., accelaration, verlocity, and rotation
 *       values of the physical joints.
 */
class Spline {
public:
    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as spline movement.
     *
     * @param _start_cfg {@ref Configruation} of the starting point of the path
     * @param _end_cfg {@ref Configruation} of the target point of the path
     * @return {@ref Trajectoy} for the movement of the robot
     */
    Trajectory* get_spline_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg);
};


#endif //SDRI_CTRL2019_SPLINE_H