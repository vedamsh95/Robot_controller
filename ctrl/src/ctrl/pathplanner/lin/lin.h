#ifndef SDRI_CTRL2019_LIN_H
#define SDRI_CTRL2019_LIN_H

#include <Configuration.h>
#include <Trajectory.h>
#include <SixDPos.h>
#include <iostream>
#include <cmath>
#include <Robot.h>
#include <functional>
#include <cassert>
#include "../ptp/ptp.h"
#include "../ptp/Single_trajectory.h"
#include "../ptp/Max_vel_trajectory.h"
#include "../ptp/Trapezoidal_trajectory.h"
#include "../../kinematics/direct/fw_kinematics.h"
#include "../../kinematics/inverse/inverse_kinematics.h"
#include "../../kinematics/inverse/IVMovement.h"
#include "../../../matplotlib-cpp-master/matplotlibcpp.h"


/**
 * This class implements the lin movement from to given {@ref Configuration}, one for the start configuration and one
 * for the target configuration.
 *
 * TODO: ensure that you always stay within the physical limits of the robot, i.e., accelaration, verlocity, and rotation
 *       values of the physical joints.
 */
class Lin {
    
private:
    Robot* robot;
    bool plot; 
    Trajectory* trajectory;
    InvKinematics* invK;
    FwKinematics* fwK;
    IVMovement* ivM;
    Ptp* ptp;
    
public:
    /**
         * Default constructor
         */
    Lin();

    /**
     * Computes a trajectory for a lin movement from a start and target configuration with a trapezoidal
     * velocity profile.
     *
     * @param _start_cfg start {@ref Configuration}
     * @param _end_cfg   target {@ref Configuration}
     * @param velocity The velocity at which the robot should move along the spline
     * @param acceleration The acceleration at the start and at the end of the movement
     * @param loopPoints  An empty vector that will be filled with the loop points for plotting purposes.
     *                    Use a nullptr if you do not need that functionality.
     * @return reference of a {@ref Trajectory} for the movement
     *         NOTE: In case the movement could not be finished, a zero configuration got added to the trajectory!
     */
    Trajectory* get_lin_trajectory(Configuration* _start_cfg, Configuration* _end_cfg, double velocity, double acceleration, std::vector<std::vector<SixDPos*>>* loopPoints);
    
    /**
     * Checks for a given configuration whether it is feasible and if not,
     * it changes the values to the limits of the robot.
     *
     * @param cfg Configuration to check and change
     */
    void makeFeasible(Configuration *cfg);
    
    /**
     * Returns  PosA + factor * (PosB-PosA) for the orientation (x, y, z)
     * but keeps orientation (A, B, C)  from PosA
     *
     * @param PosA SixDPos to keep orientation from
     * @param PosB SixDPos to add position from 
     *
     */    
    SixDPos* NextPos(SixDPos *PosA, SixDPos *PosB, double factor);
    
    /**
     * Returns configuration of given SixDPos
     *
     * @param SixDPos SixDPos to convert to configuration
     * @param StartConfig Config that result should be close to
     *
     */
    Configuration* GetConfigurations(SixDPos* SixDPos, Configuration* StartConfig);
    
    /**
     * Plots a 3D plot of given SixDPoses and the corresponding velocities
     * calculated by the distance inbetween the SixDPos.
     *
     * @param SixDPoses  vector of SixDPos to plot
     */
    
    static void Plot(vector<SixDPos*> SixDPoses);
    
};


#endif //SDRI_CTRL2019_LIN_H
