#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include "fw_kinematics.h"


SixDPos *FwKinematics::get_fw_kinematics(Configuration *_cfg) {
    // Euler angle test
    //TMatrix mat(0.8212351, -0.353276, 0.5694953, 0, 0.1512832, 0.9758426, -0.1576219, 0, -0.5501693, 0.2155997, 0.8067406, 0, 0, 0, 0, 1);
    //vector<double> angles = get_euler_angles(mat);
    //cout << angles[0] << ", " << angles[1] << ", " << angles[2] << endl;

    // Matrix multiplication test
    //TMatrix A(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 0, 0, 0, 1);
    //TMatrix B(4, 3, 2, 1, 4, 3, 2, 1, 4, 3, 2, 1, 0, 0, 0, 1);
    //TMatrix C = A.multiply(B);
    //cout << C << endl;

    // Output the cfg values:
    //cout << (*_cfg)[0] << ", " << (*_cfg)[1] << ", " << (*_cfg)[2] << ", " << (*_cfg)[3] << ", " << (*_cfg)[4] << ", " << (*_cfg)[5] << ", " << endl;

    // These are the Denavit-Hartenberg parameters for our robot
    double dh_table[7][4] = {
            {0,       M_PI,   0,    645},
            {0,       M_PI_2, 330,  0},
            {0, 0,            1150, 0},
            {-M_PI_2, M_PI_2, 115,  0},
            {0, -M_PI_2,      0,    -1220},
            {0,       M_PI_2, 0,    0},
            {M_PI,    M_PI,   0,    -215}
    };

    // Transformation matrix from the base coordinate system
    // to the coordinate system of the first joint. This does
    // not depend on any variables.
    TMatrix A = create_single_t_matrix(
            dh_table[0][0],
            dh_table[0][1],
            dh_table[0][2],
            dh_table[0][3]
    );

    // Create and multiply the individual transformation matrices
    // from one joint to the next one.
    for (int i = 0; i < 6; i++) {
        TMatrix next = create_single_t_matrix(
                dh_table[i + 1][0] + (*_cfg)[i],
                dh_table[i + 1][1],
                dh_table[i + 1][2],
                dh_table[i + 1][3]
        );
        A = A.multiply(next);
    }

    // A now contains the final transformation matrix denoting the
    // transformation from the base coordinate system to the coordinate
    // system of the flange.

    vector<double> euler_angles = get_euler_angles(A);

    // Print the results
    //cout << A << endl;
    //cout << "X: " << A.get(0, 3) << endl;
    //cout << "Y: " << A.get(1, 3) << endl;
    //cout << "Z: " << A.get(2, 3) << endl;
    //cout << "Roll  : " << euler_angles[0] << endl;
    //cout << "Pitch : " << euler_angles[1] << endl;
    //cout << "Yaw   : " << euler_angles[2] << endl;

    return new SixDPos(
            A.get(0, 3),
            A.get(1, 3),
            A.get(2, 3),
            euler_angles[0],
            euler_angles[1],
            euler_angles[2]
    );
}

TMatrix FwKinematics::create_single_t_matrix(double theta, double alpha, double r, double d) {
    double t00 = cos(theta);
    double t01 = -sin(theta) * cos(alpha);
    double t02 = sin(theta) * sin(alpha);
    double t03 = r * cos(theta);
    double t10 = sin(theta);
    double t11 = cos(theta) * cos(alpha);
    double t12 = -cos(theta) * sin(alpha);
    double t13 = r * sin(theta);
    double t20 = 0;
    double t21 = sin(alpha);
    double t22 = cos(alpha);
    double t23 = d;
    double t30 = 0;
    double t31 = 0;
    double t32 = 0;
    double t33 = 1;

    return {
            t00, t01, t02, t03,
            t10, t11, t12, t13,
            t20, t21, t22, t23,
            t30, t31, t32, t33
    };
}

vector<double> FwKinematics::get_euler_angles(TMatrix &tMatrix) {
    double *data = tMatrix.get_matrix();
    double phi, theta, psi;

    auto T = [data](int row, int col) {
        return data[(row - 1) * 4 + col - 1];
    };

    double epsilon = 0.00174532925;    // 0.1 degrees

    if (abs(T(1, 1)) < epsilon && abs(T(2, 1)) < epsilon) {
        phi = asin(-T(1, 2));
        theta = -T(3, 1) * M_PI * 0.5;
        psi = 0;
    } else {
        phi = atan2(T(2, 1), T(1, 1));
        theta = atan2(-T(3, 1), sqrt(pow(T(3, 2), 2) + pow(T(3, 3), 2)));
        psi = atan2(T(3, 2), T(3, 3));
    }
    return vector<double>({phi, theta, psi});
}
