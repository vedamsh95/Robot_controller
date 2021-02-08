#ifndef SDRI_CTRL2019_SPLINE_H
#define SDRI_CTRL2019_SPLINE_H


#include <Trajectory.h>
#include <SixDPos.h>

/**
 * This class implementes the ptp movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., accelaration, verlocity, and rotation
 *       values of the physical joints.
 */
class Spline {
public:
    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as ptp movement.
     *
     * @param _start_cfg {@ref Configruation} of the starting point of the path
     * @param _end_cfg {@ref Configruation} of the target point of the path
     * @return {@ref Trajectoy} for the movement of the robot
     */
    Trajectory* get_spline_trajectory(Configuration* start, vector<SixDPos*> path);

    SixDPos *get_spline_at(double progress);

    bool pathOnly = false;
};


#endif //SDRI_CTRL2019_SPLINE_H
