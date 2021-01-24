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



/** \brief computes the inverse kinematics
 * This class is intended to handle the computation of the inverse kinematics. This includes both the computation of the
 * configurations for the position and the orientation.
 *
 */
class InvKinematics {
private:
    Robot *robot;
    static constexpr double singularityMargin = 0.05;
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
     * @return  a reference to a <code>vector<code> containing all possible configurations for the passed position
     */
    vector<Configuration*>* get_inv_kinematics(SixDPos* _pos);
    
    /**
     * Calculation of the configurations of wrist joints in case of wrist singularity (theta5 = 0).
     * The sum of theta4 and theta6 is calculated from the matrix R36.
     * theta4+theta5 = arccos(R36[1][1])
     * and assigns to both of them the half.
     *
     * @param _R36 {@TMatrix} to compute the wrist configurations for
     * @return a reference to a array containing the configurations
     */
    array<double, 3>* CalculateSingularity(TMatrix R36);
};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
