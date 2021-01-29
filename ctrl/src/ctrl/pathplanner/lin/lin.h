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
 * This class implementes the lin movement from to given {@ref Configuration}, one for the start configuration and one
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
    
public:
    /**
         * Default constructor
         */
    Lin();
    
    /**
     * Example function computing the trajectory for a given path (defined by two configurations) as lin movement.
     *
     * @param _start_cfg {@ref Configuration} of the starting point of the path
     * @param _end_cfg {@ref Configuration} of the target point of the path
     * @param velocity The velocity for the lin movement used in the corresponding trajectory
     * @param acceleration The acceleration for lin movement used in the corresponding trajectory
     * @param Vector of loop points that cannot be traversed correctly - Will be filled
     * @return {@ref Trajectory} for the movement of the robot
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
     *Returns closest configuration out of a vector of configurations to a given configuration
     *Difference to IVMovement::GetClosestConfiguration: the first 3 joints have a higher weight. 
     *
     * @param Configurations
     * @param PrevConfig  Configuration that others need to be close to
     *
     */
    Configuration* GetClosestConfiguration(vector<Configuration*>* Configurations, Configuration* PrevConfig);
    
    static void PlotVelocity(vector<SixDPos*> SixDPoses);
    
};


#endif //SDRI_CTRL2019_LIN_H
