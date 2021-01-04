#define _USE_MATH_DEFINES

#include <cmath>
#include "fw_kinematics.h"


SixDPos *FwKinematics::get_fw_kinematics(Configuration *_cfg) {
    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.

    // -> We're using rad and mm!

    array<double, 6> &config = _cfg->get_configuration();

    // Denavit Hartenberg Method: can be used to get the position of the end effector

    double denavitHartenbergTable[7][4] = {
            {0,                       1 * M_PI,    0,    645},
            {config[0],               0.5 * M_PI,  330,  0},
            {config[1],               0,           1150, 0},
            {-0.5 * M_PI + config[2], 0.5 * M_PI,  115,  0},
            {config[3],               -0.5 * M_PI, 0,    -1220},
            {config[4],               0.5 * M_PI,  0,    0},
            {1 * M_PI + config[0],    1 * M_PI,    0,    -215},
    };

    TMatrix transformationMatrix = TMatrix(
            denavitHartenbergTable[0][0],
            denavitHartenbergTable[0][1],
            denavitHartenbergTable[0][2],
            denavitHartenbergTable[0][3]
    );
    for (int i = 1; i < 7; i++) {
        transformationMatrix = transformationMatrix.multiply(TMatrix(
                denavitHartenbergTable[i][0],
                denavitHartenbergTable[i][1],
                denavitHartenbergTable[i][2],
                denavitHartenbergTable[i][3]
        ));
    }

    transformationMatrix.print();

    // Euler Angles: can be used to get the rotation of the end effector

    return new SixDPos(
            transformationMatrix.get(0, 3),
            transformationMatrix.get(1, 3),
            transformationMatrix.get(2, 3),
            0, M_PI, 0
    );
}
