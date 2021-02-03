#define _USE_MATH_DEFINES

#include "sdir_ctrl.h"
#include "kinematics/direct/fw_kinematics.h"
#include "kinematics/inverse/inverse_kinematics.h"
#include "pathplanner/ptp/ptp.h"
#include "pathplanner/lin/lin.h"
#include "pathplanner/spline/spline.h"
#include <Trajectory.h>
#include <cfloat>
#include <iostream>

SixDPos* SdirCtrl::get_pos_from_config(Configuration* _cfg)
{
    //ToDo: IMPLEMENT!
    FwKinematics fwKinematics;
    SixDPos* new_pos = fwKinematics.get_fw_kinematics(_cfg);
    return new_pos;
}


vector<Configuration*>* SdirCtrl::get_config_from_pos(SixDPos* pos)
{
    // TODO: we need the current configuration - it might be in line 70 of the script
    //ToDo: IMPLEMENT!
    InvKinematics invKinematics;
    vector<Configuration*>* new_cfg = invKinematics.get_inv_kinematics(pos);
    return new_cfg;
}

const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
Configuration* SdirCtrl::get_next_config_from_pos(Configuration* previous, SixDPos* pos)
{
    cout << "Old config: " << (*previous)[0] << " " << (*previous)[1] << " " << (*previous)[2] << " " << (*previous)[3] << " " << (*previous)[4] << " " << (*previous)[5] << endl;
    InvKinematics invKinematics;
    vector<Configuration*>* new_cfg = invKinematics.get_inv_kinematics(pos);
    Configuration* bestConfiguration = nullptr;
    double bestDistance = DBL_MAX;
    for (int i = 0; i < new_cfg->size(); i++) {
        double distance = 0;
        for (int j = 0; j < 6; j++) {
            distance += abs((*(*new_cfg)[i])[j] - (*previous)[j]) * MAX_VELOCITY[j];
        }
        if (distance < bestDistance) {
            bestDistance = distance;
            bestConfiguration = (*new_cfg)[i];
        }
    }
    cout << "New config: " << (*bestConfiguration)[0] << " " << (*bestConfiguration)[1] << " " << (*bestConfiguration)[2] << " " << (*bestConfiguration)[3] << " " << (*bestConfiguration)[4] << " " << (*bestConfiguration)[5] << endl;

    return bestConfiguration;
}


Trajectory* SdirCtrl::move_robot_ptp(Configuration* start, SixDPos* end, bool sync)
{
    // not used by the UI
    return move_robot_ptp(start, get_next_config_from_pos(start, end), sync);
}

Trajectory* SdirCtrl::move_robot_ptp(Configuration* start, Configuration* end, bool sync)
{
    Ptp ptp;
    return ptp.get_ptp_trajectory(start, end, sync);
}


Trajectory* SdirCtrl::move_robot_lin(Configuration* start, SixDPos* end)
{
    // not used by the UI
    return move_robot_lin(start, get_next_config_from_pos(start, end));
}


Trajectory* SdirCtrl::move_robot_lin(Configuration* start, Configuration* end)
{
    //ToDo: IMPLEMENT!
    Lin lin;
    return lin.get_lin_trajectory(start, end);
}

Trajectory *SdirCtrl::move_robot_spline(vector<SixDPos *> path) {
    Spline spline;
    return spline.get_spline_trajectory(path);
}
