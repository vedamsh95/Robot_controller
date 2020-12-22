#define _USE_MATH_DEFINES

#include <math.h>
#include "fw_kinematics.h"


SixDPos* FwKinematics::get_fw_kinematics(Configuration *_cfg)
{
    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.

    array<double, 6> &config = _cfg->get_configuration();
    
    double m_transformation[4][4];

    return new SixDPos(1.757, 0.0, 1.91, 0, M_PI, 0);
}
