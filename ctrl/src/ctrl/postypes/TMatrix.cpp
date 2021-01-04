#include "TMatrix.h"
#include <math.h>
#include <cstdio>

//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!

TMatrix::TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven,
                 double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen,
                 double _fourteen, double _fifteen, double _sixteen) {
    m_transformation[0][0] = _one;
    m_transformation[0][1] = _two;
    m_transformation[0][2] = _three;
    m_transformation[0][3] = _four;
    m_transformation[1][0] = _five;
    m_transformation[1][1] = _six;
    m_transformation[1][2] = _seven;
    m_transformation[1][3] = _eight;
    m_transformation[2][0] = _nine;
    m_transformation[2][1] = _ten;
    m_transformation[2][2] = _eleven;
    m_transformation[2][3] = _twelve;
    m_transformation[3][0] = _thirteen;
    m_transformation[3][1] = _fourteen;
    m_transformation[3][2] = _fifteen;
    m_transformation[3][3] = _sixteen;
}


TMatrix::TMatrix(double _trans[6]) : TMatrix(_trans[0], _trans[0], _trans[0], _trans[0], _trans[0], _trans[0]) {}

// General constuctor
TMatrix::TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z) {
    m_transformation[0][0] = cos(_rot_z) * cos(_rot_y);
    m_transformation[0][1] = sin(_rot_z) * cos(_rot_y);
    m_transformation[0][2] = -sin(_rot_y);

    m_transformation[1][0] = cos(_rot_z) * sin(_rot_y) * sin(_rot_x) - sin(_rot_z) * cos(_rot_y);
    m_transformation[1][1] = sin(_rot_z) * sin(_rot_y) * sin(_rot_x) + cos(_rot_z) * cos(_rot_y);
    m_transformation[1][2] = cos(_rot_y) * sin(_rot_x);

    m_transformation[2][0] = cos(_rot_z) * sin(_rot_y) * cos(_rot_x) + sin(_rot_z) * sin(_rot_y);
    m_transformation[2][1] = sin(_rot_z) * sin(_rot_y) * cos(_rot_x) - cos(_rot_z) * sin(_rot_y);
    m_transformation[2][2] = cos(_rot_y) * sin(_rot_x);

    m_transformation[3][0] = _trans_x;
    m_transformation[3][1] = _trans_y;
    m_transformation[3][2] = _trans_z;

    m_transformation[0][3] = 0;
    m_transformation[1][3] = 0;
    m_transformation[2][3] = 0;
    m_transformation[3][3] = 1;
}

// Denavit Hartenberg constructor
TMatrix::TMatrix(double _rot_theta, double _rot_alpha, double _trans_rx, double _trans_dz) {
    m_transformation[0][0] = cos(_rot_theta);
    m_transformation[0][1] = sin(_rot_theta);
    m_transformation[0][2] = 0;

    m_transformation[1][0] = -sin(_rot_theta) * cos(_rot_alpha);
    m_transformation[1][1] = cos(_rot_theta) * cos(_rot_alpha);
    m_transformation[1][2] = sin(_rot_alpha);

    m_transformation[2][0] = sin(_rot_theta) * sin(_rot_alpha);
    m_transformation[2][1] = -cos(_rot_theta) * sin(_rot_alpha);
    m_transformation[2][2] = cos(_rot_alpha);

    m_transformation[3][0] = _trans_rx * cos(_rot_theta);
    m_transformation[3][1] = _trans_rx * sin(_rot_theta);
    m_transformation[3][2] = _trans_dz;

    m_transformation[0][3] = 0;
    m_transformation[1][3] = 0;
    m_transformation[2][3] = 0;
    m_transformation[3][3] = 1;
}

void TMatrix::print() {
    for (int i = 0; i < 4; i++) {
        printf(
                "%8.2f %8.2f %8.2f %8.2f\n",
                m_transformation[i][0],
                m_transformation[i][1],
                m_transformation[i][2],
                m_transformation[i][3]
        );
    }
}

TMatrix TMatrix::multiply(TMatrix b) {
    double c[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int p = 0; p < 4; p++) {
                c[i][j] += m_transformation[i][p] * b.m_transformation[p][j];
            }
        }
    }
    return TMatrix(
            c[0][0], c[0][1], c[0][2], c[0][3],
            c[1][0], c[1][1], c[1][2], c[1][3],
            c[2][0], c[2][1], c[2][2], c[2][3],
            c[3][0], c[3][1], c[3][2], c[3][3]
    );
}

