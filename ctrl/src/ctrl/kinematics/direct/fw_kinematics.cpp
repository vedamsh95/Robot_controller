#define _USE_MATH_DEFINES

#include <math.h>
#include "fw_kinematics.h"


SixDPos* FwKinematics::get_fw_kinematics(Configuration *_cfg)
{

    double* M0 = D_H_matrix(0, M_PI, 0, 645);

    double phi1 = _cfg->get_configuration()[0];

    double* M1 = D_H_matrix(phi1, M_PI, 330, 0);

    double* M0x1 = matrix_mult(M0, M1);

    double phi2 = _cfg->get_configuration()[1];

    double* M2 = D_H_matrix(phi2, 0.5 * M_PI, 1150, 0);

    double* M0x1x2 = matrix_mult(M0x1, M2);

    double phi3 = _cfg->get_configuration()[2];

    double* M3 = D_H_matrix(phi3 - 0.5 * M_PI, 0.5 * M_PI, 115, 0);

    double* M0x1x2x3 = matrix_mult(M0x1x2, M3);

    double phi4 = _cfg->get_configuration()[3];

    double* M4 = D_H_matrix(phi4, -0.5 * M_PI, 0, -1220);

    double* M0x1x2x3x4 = matrix_mult(M0x1x2x3, M4);

    double phi5 = _cfg->get_configuration()[4];

    double* M5 = D_H_matrix(phi5, 0.5 * M_PI, 0, 0);

    double* M0x1x2x3x4x5 = matrix_mult(M0x1x2x3x4, M5);

    double phi6 = _cfg->get_configuration()[5];

    double* M6 = D_H_matrix(0.5 * M_PI + phi6, 0.5 * M_PI, 0, -215);

    double* M_fin = matrix_mult(M0x1x2x3x4x5, M6);

    return new SixDPos(M_fin[3], M_fin[7], M_fin[11], 0, 0, 0);


    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.

    //return new SixDPos(1.757, 0.0, 1.91, 0, M_PI, 0);
}

double* FwKinematics::D_H_matrix(double angle, double a, double r, double d) {

    double p[16];

    p[0] = cos(angle);

    p[1] = -sin(angle) * cos(a);

    p[2] = sin(angle) * sin(a);

    p[3] = r * cos(angle);

    p[4] = sin(angle);

    p[5] = cos(angle) * cos(a);

    p[6] = -cos(angle) * sin(a);

    p[7] = r * sin(angle);

    p[8] = 0;

    p[9] = sin(a);

    p[10] = cos(a);

    p[11] = d;

    p[12] = 0;

    p[13] = 0;

    p[14] = 0;

    p[15] = 1;

    return p;


}


double* FwKinematics::matrix_mult(double* mat_a, double* mat_b) {

    double mat[16];

    double sum = 0;

    for (int i=0; i < 4; i++) {
        for (int k=0; k < 4; k++) {
            for (int l = 0; l < 4; l++) {
                sum = sum + mat_a[4 * i + l] * mat_b[4 * l + k];
            }
            mat[4 * i + k] = sum;
            sum = 0;
        }
    }

    return mat;


}
