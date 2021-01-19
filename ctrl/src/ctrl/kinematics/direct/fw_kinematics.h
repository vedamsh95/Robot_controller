#ifndef SDRI_CTRL2019_FW_KINEMATICS_H
#define SDRI_CTRL2019_FW_KINEMATICS_H


#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>

/**
 * This class is intended to handle the direct kinematic computation from a given configuration.
 * It makes use of Roll, Pitch, Yaw Euler angles:
 * Extrinsic:
 *  Yaw around z, Pitch around y', Roll around z''
 * Intrinsic:
 *  Roll around z, Pitch around y, Yaw around x
 */
class FwKinematics {
public:
    /**
     * Computes the forward kinematic from a given {@Configuration}
     *
     * @param _cfg {@Configuration} to compute the forward kinematic from
     * @return computed {@ref SixDPos}
     */
    SixDPos *get_fw_kinematics(Configuration *_cfg);
    
    /**
    * Creates the transformation matrix for a given set of DNH parameters
    *
    * @param theta [rad]
    * @param alpha [rad]
    * @param r [mm]
    * @param d [mm]
    * @return Transformation matrix {TMatrix}
    */
    TMatrix create_single_t_matrix(double theta, double alpha, double r, double d);

private:
    /**
    * Calculates the Roll-Pitch-Yaw Euler angles for a given homogeneous
    * transformation matrix.
    *
    * @param Homogeneous transformation matrix
    * @return Set of Euler Angles denoting Roll, Pitch, and Yaw [rad]
    */
    vector<double> get_euler_angles(TMatrix &tMatrix);

};


#endif //SDRI_CTRL2019_FW_KINEMATICS_H
