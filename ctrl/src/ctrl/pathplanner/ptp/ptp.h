#ifndef SDRI_CTRL2019_PTP_H
#define SDRI_CTRL2019_PTP_H

#include <Trajectory.h>
#include <Robot.h>
#include <functional>
#include <math.h>
#include <iostream>
#include <cassert>
#include "Single_trajectory.h"
#include "Max_vel_trajectory.h"
#include "Trapezoidal_trajectory.h"
#include "../../matplotlib-cpp-master/matplotlibcpp.h"

/**
 * This class implements the ptp movement from two given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 */
class Ptp {

private:
    Robot *robot;
    Trajectory *trajectory;
    bool plot;

public:
    /**
     * Default constructor
     */
    Ptp();

    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as ptp movement.
     *
     * @param _start_cfg {@ref Configuration} of the starting point of the path
     * @param _end_cfg {@ref Configuration} of the target point of the path
     * @param sync Whether the motion should be synchronous
     * @return {@ref Trajectory} for the movement of the robot
     */
    Trajectory *get_ptp_trajectory(Configuration *_start_cfg, Configuration *_end_cfg, bool sync);

    /**
     * Checks for a given configuration whether it is feasible and if not,
     * it changes the values to the limits of the robot.
     *
     * @param cfg Configuration to check and change
     */
    void make_feasible(Configuration *cfg);

    /**
     * Plots the given configuration using the matplotlibcpp library to visualise the trajectories.
     *
     * @param configs     The configurations of the ptp movement to plot
     */
    static void plot_movement(vector<Configuration *> &configs);

};


#endif //SDRI_CTRL2019_PTP_H
