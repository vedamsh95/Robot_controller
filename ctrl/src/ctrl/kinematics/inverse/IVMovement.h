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

#define SINGULARITY_MARGIN 0.075
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
     * @param loopPoints  Points where Configurations needed to be interpolated (can also be a nullptr)
     * @param end_cfg         (optional) last configuration
     * @return trajectory along the given SixDPoses
     */
    Trajectory* getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg, std::vector<std::vector<SixDPos*>>* loopPoints, Configuration* end_cfg = nullptr);
    
    
    /**
     * Calculates closest configuration from a vector containig configuration to a given configuration.
     * Weights can be added if the resemblance of the first three joints is more important than the resemblance
     * of the wrist joints.
     *
     * @param _configs  vector containing possible configurations.
     * @param _prevConfig  reference configuration
     * @param weight (optional) true if arm joints are more important
     *
     * @return the closest configuration
     */
    Configuration* GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig, bool weight = false);

private:
    InvKinematics *invK;
    FwKinematics* fwK;
    Trajectory *trajectory;
    Robot *robot;
    std::vector<std::vector<SixDPos*>>* loopVector;
    
    
    /**
     * Calculates if joint velocities are too high so that _positions and trajectory can be changed accordingly.
     * If veocities are too high SixDPos are added to _positions and belonging Configurations are inserted in trajectory.
     *
     * @param trajectory  trajectory to adjust if velocities to high.
     * @param _positions  SixDPoses to interpolate to reduce velocity.
     */
    void CheckVelocities(Trajectory* _trajectory, vector<SixDPos*>* _positions);
    
    /**
     * Calculates closest configuration from a vector containig configuration to a given configuration.
     * The first three joints of the reference configuration are extracted from _prevConfig, the wrist
     * joints are taken from _wristConfig. The resemblance of the arm joints is prioritized.
     *
     * @param _configs  vector containing possible configurations.
     * @param _prevConfig  reference configuration first three joints
     * @param _wristConfig reference configuration wrist joints
     * @return the closest configuration
     */
    Configuration* GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig, Configuration* _wristConfig );
    
    /**
     * Computes the configuration of a new SixDPos C that is interpolated from the SixDPos _position->at(index) and _position->at(index-1).
     * Only interpolates the X, Y and Z values. The new SixDPos is inserted in _positions.
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
     * @param width length of the singularity
     */
    void wsInterpolation(Trajectory* trajectory, int width);

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
     * Computes value at wrist singularity if the last available configuration is at singularity and therefore it can not be interpolated.
     * Calculation is made with the difference of the sums of theta4 and theta6. 
     *
     * @param Config  Configuration that needs to be adjusted
     * @param width    Width of the singularity region.
     *
     */
    void wsLastConfig(Configuration* Config, int width);
    
    /**
     * Computes configurations at overhead singularities ( x & y ==0) by interpolating theta1 from values outside of singularity trajectory(-length) and trajectory(-1)). Recalculates the wrist joints for the new configuratin of the first 3 joints.
     * Corresponding values are changed in trajectory.
     *
     * @param trajectory  trajectory containing configurations
     * @param width length of the singularity
     * @param _positions positions corresponding to trajectory
     */
    
    void osInterpolation(Trajectory* trajectory, int width, vector<SixDPos*>* _positions);
    
    /**
     * Computes configurations at overhead singularities ( x & y ==0) by interpolating theta1  from two given values (startConfig & endConfig). And recalculates the wrist joints accordingly.
     *
     * @param startConfig Configuration before singularity.
     * @param curConfig Configuration at singularity.
     * @param endConfig Configuration after singularity.
     * @param posA SixDPos before singularity.
     * @param posB SixDPos at singularity.
     * @param posC SixDPos after singularity.
     *
     * @return a reference containing the interpolated configuration.
     */
    Configuration* osInterpolation(Configuration* startConfig, Configuration* curConfig, Configuration* endConfig, SixDPos* posA, SixDPos* posB, SixDPos* posC);
    
    /**
     * Computes value at overehad singularity if the last available configuration is at singularity and therefore can not be interpolated.
     * Theta 1 is set to last known theta1 before the singularity and the wrist joints are adjusted accordingly.
     * The configuration is changed in trajectory.
     *
     * @param Config  Configuration that needs to be adjusted
     * @param width    Width of the singularity region
     * @param Pos         SixDPos of the singularity
     *
     */
    void osLastConfig(Configuration* Config, SixDPos* Pos, int width);
    
    /**
     * Computes new sixDPos at elbow singularity.
     *
     * @param _pos current positon     
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


