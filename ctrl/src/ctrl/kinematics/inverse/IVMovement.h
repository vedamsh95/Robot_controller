#pragma once
#include <Configuration.h>
#include <Trajectory.h>
#include <SixDPos.h>
#include "inverse_kinematics.h"
#include "../direct/fw_kinematics.h"
#include <Robot.h>
#include <iostream>
#include <math.h>
#include <cmath>

#define SINGULARITY_MARGIN 0.05
#define JOINT_5_MAX (49.0/72.0)*M_PI

class IVMovement
{
public:
    IVMovement();
    ~IVMovement();
    /**
     * Calculates trajectory from given SixDPos. Therefore the configuration for each SixDPos
     * that is closest to the previous configuration is calculated and checked for singularities.
     * The joint velocities of the trajectory are examined and if necessary adjusted.
     * If the trajectory can not be calculated conitniously the adjustments still calculated and
     * displayed.
     *
     * @param _positions  SixDPoses of path that is going to be calculated
     * @param start_cfg    First configuration of trajectory
     * @param loopPoints  Points where Configurations needed to be interpolated
     * @param end_cfg         (optional) last configuration
     * @return trajectory along the given SixDPoses
     */
    Trajectory* getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg, std::vector<std::vector<SixDPos*>>* loopPoints, Configuration* end_cfg = nullptr);
    
    /**
     * Calculates if joint velocities are too high so that _positions can be changed accordingly.
     * If veocities are too high SixDPos are added to _positions and belonging Configurations are inserted in trajectory.
     *
     * @param trajectory  trajectory to adjust if velocities to high.
     * @param _positions  SixDPoses to interpolate to reduce velocity.
     */
    void CheckVelocities(Trajectory* _trajectory, vector<SixDPos*>* _positions);

private:
	Configuration* GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig);
    
	InvKinematics *invK;
    FwKinematics* fwK;
	Trajectory *trajectory;
    Robot *robot;
    std::vector<std::vector<SixDPos*>>* loopVector;
    
    /**
     * Computes the configuration of a new SixDPos C that is interpolated from the SixDPos _position->at(index) and _position->at(index-1).
     * Only interpolatex the X, Y and Z values. The new SixDPos is inserted in _positions.
     * The returned configuration is the one closest to trajectory->at(index-1).
     *
     * @param trajectory  trajectory to get previous configurations from.
     * @param _positions vector containing sixDposes to interpolate
     * @param index index of _positions that needs to be interpolated
     * @return true if values were interpolated and inserted
     */
     bool Interpolate(Trajectory* trajectory, vector<SixDPos*>* _positions, int index);
    
    /**
     * Computes configurations at wrist singularities (theta5  == 0) by interpolating theta4 and theta6 from values outside of singularity (trajectory(-length) and trajectory(-1)).
     * And changes corresponding values in trajectory.
     *
     * @param trajectory  trajectory containing configurations
     * @param length length of the singularity
     */
    void wsInterpolation(Trajectory* trajectory, int length);

    /**
     * Computes one single configuration at wrist singularities (theta5  == 0) by interpolating theta4 and theta6 from two given values (startConfig & endConfig).
     *
     * @param startConfig  Configuration before singularity.
     * @param curConfig  singularity configuration.
     * @param endConfig  Configuration after singularity.
     *
     * @return a reference containing the interpolated configuration.
     */
    Configuration* wsInterpolation(Configuration* startConfig, Configuration* curConfig, Configuration* endConfig);

    
    /**
     * Computes configurations at overhead singularities ( x & y ==0) by interpolating theta1 from values outside of singularity trajectory(-length) and trajectory(-1)).
     * And changes corresponding values in trajectory.
     *
     * @param trajectory  trajectory containing configurations
     * @param length length of the singularity
     */
    //void osInterpolation(Trajectory* trajectory, int lengths, vector<SixDPos*>* _positions);
    
    void osInterpolation(Trajectory* trajectory, int lengths, vector<SixDPos*>* _positions);
    
    /**
     * Computes configurations at overhead singularities ( x & y ==0) by interpolating theta1  from two given values (startConfig & endConfig).
     *
     * @param startConfig  Configuration before singularity.
     * @param curConfig  singularity configuration.
     * @param endConfig  Configuration after singularity.
     *
     * @return a reference containing the interpolated configuration.
     */
    Configuration* osInterpolation(Configuration* startConfig, Configuration* curConfig, Configuration* endConfig, SixDPos* posA, SixDPos* posB, SixDPos* posC);
    
    /**
     * Computes new sixDPos at elbow singularity.
     *
     * @param _pos current positon
     *
     * @return reference containing the adjusted SixDPos
     */
    SixDPos* esCalculation(SixDPos* _pos);
    
    /**
     * Checks if configuration _config is wrist singularity
     */
    bool wristSingularity(Configuration* _config);
    
    /**
     * Checks if position _pos is overhead singularity
     */
    bool overheadSingularity(SixDPos* _pos);
    
    /**
     * Checks if position _pos is overhead singularity
     */
    bool elbowSingularity(SixDPos* _pos);
    
    /**
     * Computes the configuration  that is interpolated from the Configurations  trajectory->at(index) and trajectory->at(index-1).
     *
     * @param trajectory  trajectory to get previous configurations from.
     * @param index index of _positions that needs to be interpolated
     * @return true if values were interpolated and inserted
     */
    bool JointInterpolate(Trajectory* trajectory, int index);
    
    
    /**
     * Gets distance inbetween Configuration* A and Configuration* B
     */
    double distance(Configuration* A, Configuration* B);
    
    /**
     * Gets maximal distance distance inbetween Configuration* A and Configuration* B
     */
    double getMaxDiff(Configuration* ConifgA, Configuration* ConfigB);
    
    
    /**
     * Stores the interpolated poins in loopPoints. 
     */
    void getLoopPoints(std::vector<int> LoopStart, std::vector<int> LoopSize);
    
    
};


