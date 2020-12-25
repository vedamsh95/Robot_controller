#ifndef SDRI_CTRL2019_FW_KINEMATICS_H
#define SDRI_CTRL2019_FW_KINEMATICS_H


#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>

/**
 * This class is intended to handle the direct kinematic computation from a given configuration
 */
class FwKinematics {
public:
    /**
     * Computes the forward kinematic from a given {@Configuration}
     * @param _cfg {@Configuration} to compute the forward kinematic from
     * @return computed {@ref SixDPos}
     */
    SixDPos* get_fw_kinematics(Configuration* _cfg);

    double* D_H_matrix(double angle, double a, double r, double d);

    double* matrix_mult(double* mat_a, double* mat_b);

};


#endif //SDRI_CTRL2019_FW_KINEMATICS_H
