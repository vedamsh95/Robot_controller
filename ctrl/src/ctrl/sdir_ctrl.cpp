#define _USE_MATH_DEFINES

#include "sdir_ctrl.h"
#include "kinematics/direct/fw_kinematics.h"
#include "kinematics/inverse/inverse_kinematics.h"
#include "pathplanner/ptp/ptp.h"
#include "pathplanner/lin/lin.h"
#include <Trajectory.h>
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
    //ToDo: IMPLEMENT!
    InvKinematics invKinematics;
    vector<Configuration*>* new_cfg = new vector<Configuration*>();
    bool config_status = false;
    new_cfg = invKinematics.get_inv_kinematics(pos);
    if(new_cfg->size() != 0){
        config_status = true;
    }else{
        config_status = false;
        std::cout << "ERROR: for the given Point is no possible configuration reachable!!" << std::endl;
    }

    if(config_status){
        std::cout << new_cfg->at(0)->get_configuration().at(1) << std::endl;
    }
    return new_cfg;
}


Trajectory* SdirCtrl::move_robot_ptp(SixDPos* start, SixDPos* end)
{
    //ToDo: IMPLEMENT!
    // not needed because function is not used in main.cpp
    return NULL;
}

Trajectory* SdirCtrl::move_robot_ptp(Configuration* start, Configuration* end, bool sync)
{
    //ToDo: IMPLEMENT!
    Ptp ptp;
    return ptp.get_ptp_trajectoy(start, end, sync);
}


Trajectory* SdirCtrl::move_robot_lin(SixDPos* start, SixDPos* end, double speed, double acceleration)
{
    //ToDo: IMPLEMENT!
    // not needed because function is not used in main.cpp
    return NULL;
}


Trajectory* SdirCtrl::move_robot_lin(Configuration* start, Configuration* end, double speed, double acceleration)
{
    //ToDo: IMPLEMENT!
    Lin lin;
    return lin.get_lin_trajectoy(start, end, speed, acceleration);
}