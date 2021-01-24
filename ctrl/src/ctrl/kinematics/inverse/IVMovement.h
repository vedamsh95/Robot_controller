#pragma once
#include <Configuration.h>
#include <Trajectory.h>
#include <SixDPos.h>
#include "inverse_kinematics.h"
#include <Robot.h>
#include <iostream>
#include <math.h>
#include <cmath>

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
     * @return a reference containing the new configuration.
     */
    Configuration * Interpolate(Trajectory* trajectory, vector<SixDPos*>* _positions, int index);
    
    /**
     * Computes configurations at wrist singularities (theta5  == 0) by interpolating theta4 and theta6 from values outside of singularity (startConfig and last configuration in trajectory).
     *
     * @param startConfig  Configuration before singularity.
     * @param trajectory  Configruations in singularity (last configuration is the first configuration outside of the singularity)
     * @return a reference containing the trajectory of the interpolated configurations.
     */
    Trajectory* wsInterpolation(Configuration* startConfig, Trajectory* trajectory);
    
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

};


