#ifndef SDRI_CTRL2019_INVERSE_KINEMATICS_H
#define SDRI_CTRL2019_INVERSE_KINEMATICS_H


#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>
#include "IVKinPos.h"
#include <Robot.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "../direct/fw_kinematics.h"

#define JOINT_5_MAX (49.0/72.0)*M_PI

/** \brief computes the inverse kinematics
 * This class is intended to handle the computation of the inverse kinematics. This includes both the computation of the
 * configurations for the position and the orientation.
 *
 */
class InvKinematics {
private:
    Robot *robot;
    vector<Configuration*>* solutions;
    static constexpr double singularityMargin = 0.05;
    
    /**
     * This method sets theta 5 to the boundaries (0 or JOINT_5_MAX) if the values are too high/low.
     *
     * @param theta5 {@ref double} configuration of theta 5
     * @param j position in configuration matrix to decide weather theta 5 has to be > or < 0
     */
    void setToLimits(double* theta5, int j);

    
public:
    /**
     * Default constructor
     */
    InvKinematics();
    /**
     * This method computes all available configurations for a given position and returns them as a vector. It is the
     * main API function for the inverse kinematics computation. It should coordinate the handling between computing the
     * configuration for the position of the wrist point and the orientation at the tcp.
     *
     * @param _pos {@ref SixDPos} to compute the configuration for
     * @param setJointLimits when true values for the wrist joints are set to limits if they are exceeded.
     * @return  a reference to a <code>vector<code> containing all possible configurations for the passed position
     */
    vector<Configuration*>* get_inv_kinematics(SixDPos* _pos, bool setJointLimits = false);
    
    /**
     * Allows the calculation of all available configurations for given values for the arm joints.
     *
     * @param _pos {@ref SixDPos} to compute the configuration for
     * @param armJoint {@ref array<double, 3>}  vector containing the configurations of first three joints
     * @return  a reference to a <code>vector<code> containing all possible configurations for the passed position
     */
    vector<Configuration*>* get_inv_kinematics(SixDPos* _pos, array<double, 3>* armJoints);
    
    /**
     * This method computes all available configurations for a given Matrix R_06 and the rotation for the arm joints
     * and stores them in the solutions variable.
     *
     * @param R_06 {@ref Tmatrix} rotation matrix of all joints
     * @param ActPos  {@ref array<double, 3>} values for the configuration of the first three joints
     * @param setJointLimits when true values for the wrist joints are set to limits if they are exceeded.
     */
    void calcSolutions(TMatrix* R_06, array<double,3>* ActPos, bool setJointLimits);
    
    /**
     *
     * Calculation of the configurations of wrist joints in case of wrist singularity (theta5 == 0).
     * The sum of theta4 and theta6 is calculated from the matrix R36.
     * theta4+theta5 = arccos(R36[1][1])
     * and assigns to both of them the half.
     *
     * @param _R36 {@TMatrix} to compute the wrist configurations for
     * @return a reference to a array containing the configurations
     */
    array<double, 3>* CalculateSingularity(TMatrix R36);
    
    /**
     *
     * Signum function.
     * @param vaule value to get singum from
     */
    double sign(double value);
};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
