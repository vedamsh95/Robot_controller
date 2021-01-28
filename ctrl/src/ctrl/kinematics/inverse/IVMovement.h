#pragma once
#include <Configuration.h>
#include <Trajectory.h>
#include <SixDPos.h>
#include "inverse_kinematics.h"
#include <Robot.h>
#include <iostream>
#include <math.h>
#include <cmath>

#define SINGULARITY_MARGIN 0.05

class IVMovement
{
public:
    IVMovement();
    ~IVMovement();
    Trajectory* getMovement(vector<SixDPos *>* _positions, Configuration * start_cfg);
    
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
	Trajectory *trajectory;
    Robot *robot;
    
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
    
    Configuration* osInterpolation(Configuration* startConfig, Configuration* curConfig, Configuration* endConfig);
    
    /**
     * Computes configurations at overhead singularities ( x & y ==0) by interpolating theta1 from values outside of singularity trajectory(-length) and trajectory(-1)).
     * And changes corresponding values in trajectory.
     *
     * @param trajectory  trajectory containing configurations
     * @param length length of the singularity
     */
    void osInterpolation(Trajectory* trajectory, int length);
    
    /**
     * Gets distance inbetween Configuration* A and Configuration* B
     */
    double distance(Configuration* A, Configuration* B);
    
    /**
     * Checks if configuration _config is wrist singularity
     */
    bool wristSingularity(Configuration* _config);
    
    /**
     * Checks if position _pos is overhead singularity
     */
    bool overheadSingularity(SixDPos* _pos);
    
    
};


