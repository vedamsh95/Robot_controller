#define _USE_MATH_DEFINES

#include <math.h>
#include "fw_kinematics.h"
#include <iostream>


SixDPos* FwKinematics::get_fw_kinematics(Configuration *_cfg)
{
    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.
    //
    //TODO: difference rad to deg; final clac of xyz, RPY
    
    double new_theta1 = (*_cfg)[0];
    double new_theta2 = (*_cfg)[1];
    double new_theta3 = (*_cfg)[2];
    double new_theta4 = (*_cfg)[3];
    double new_theta5 = (*_cfg)[4];
    double new_theat6 = (*_cfg)[5];
    
//    cout << new_theta1 << new_theta3 << endl;
//    cout << _cfg << endl;

    // Initialize Parameters
    double DHC_Matrix[7][4];

    DHC_Matrix[0][0] = 0;                       // Theta (CS0 -> CS1)
    DHC_Matrix[0][1] = M_PI;                    // alpha
    DHC_Matrix[0][2] = 0;                       // r
    DHC_Matrix[0][3] = 645;                     // d

    DHC_Matrix[1][0] = new_theta1;              // CS1 -> CS2
    DHC_Matrix[1][1] = M_PI_2;
    DHC_Matrix[1][2] = 330;
    DHC_Matrix[1][3] = 0;

    DHC_Matrix[2][0] = new_theta2;              // CS2 -> CS3
    DHC_Matrix[2][1] = 0;
    DHC_Matrix[2][2] = 1150;
    DHC_Matrix[2][3] = 0;

    DHC_Matrix[3][0] = -M_PI_2 + new_theta3;    // CS3 -> CS4
    DHC_Matrix[3][1] = M_PI_2;
    DHC_Matrix[3][2] = 115;
    DHC_Matrix[3][3] = 0;
    
    DHC_Matrix[4][0] = new_theta4;              // CS4 -> CS5
    DHC_Matrix[4][1] = -M_PI_2;
    DHC_Matrix[4][2] = 0;
    DHC_Matrix[4][3] = -1220;

    DHC_Matrix[5][0] = new_theta5;              // CS5 -> CS6
    DHC_Matrix[5][1] = M_PI_2;
    DHC_Matrix[5][2] = 0;
    DHC_Matrix[5][3] = 0;

    DHC_Matrix[6][0] = M_PI + new_theat6;       // CS6 -> CS7
    DHC_Matrix[6][1] = M_PI;
    DHC_Matrix[6][2] = 0;
    DHC_Matrix[6][3] = -215;

    double* A[4][4];
    double* B[4][4];
    double* result[4][4];

    // calcualte transformation matricies
    for (int i = 0; i <= 6; i++)
    {
    
        double theta = DHC_Matrix[i][0];
        double alpha = DHC_Matrix[i][1];
        double r = DHC_Matrix[i][2];
        double d = DHC_Matrix[i][3];

        construct_transformation(theta, alpha, r, d, A);
        
        if (i == 0)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    *B[i][j] = *A[i][j];
                }
            }
        }
        else
        {
            mat_mul4x4(A, B, result);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    *B[i][j] = *result[i][j];
                }
            }
        }
    }

    // Calculate X, Y and Z form Position Matrix
    double x = *result[0][3];
    double y = *result[1][3];
    double z = *result[2][3];

    double roll = atan2(*result[1][0], *result[0][0]);
    double pitch = atan2(-*result[2][0], sqrt(pow(*result[2][1], 2) + pow(*result[2][2], 2)));
    double yaw = atan2(*result[2][1], *result[2][2]);

    cout << "x:" << x << " y:" << y << " z:" << z << endl;
    cout << "roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << endl;

    return new SixDPos(x, y, z, roll, pitch, yaw);
}


void construct_transformation(double theta, double alpha, double r, double d, double* A[4][4])
{
    *A[0][0] = cos(theta);
    *A[0][1] = -sin(theta) * cos(alpha);
    *A[0][2] = sin(theta) * sin(alpha);
    *A[0][3] = r * cos(theta);

    *A[1][0] = sin(theta);
    *A[1][1] = cos(theta) * cos(alpha);
    *A[1][2] = -cos(theta) * sin(alpha);
    *A[1][3] = r * sin(theta);

    *A[2][0] = 0;
    *A[2][1] = sin(alpha);
    *A[2][2] = cos(alpha);
    *A[2][3] = d;

    *A[3][0] = 0;
    *A[3][1] = 0;
    *A[3][1] = 0;
    *A[3][3] = 1;

}


void mat_mul4x4(double* m1[4][4], double* m2[4][4], double* result[4][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            *result[i][j] = *m1[i][0] * *m2[0][j] + *m1[i][1] * *m2[1][j] + *m1[i][2] * *m2[2][j] + *m1[i][3] * *m2[3][j];
        }
    }
}
