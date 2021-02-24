//
// Created by Tim on 24.02.2021.
//

#include "gtest/gtest.h"
#include "fw_kinematics.h"
#include "inverse_kinematics.h"

TEST(Kinematic, direct)
{
    FwKinematics fw;
    SixDPos* pos = fw.get_fw_kinematics(new Configuration({0,0,0,0,0,0}));
    std::for_each(pos->get_position().begin(), pos->get_position().end(), [](double j) {cout << j << " ";});
}

TEST(Kinematic, inverse)
{
    InvKinematics ik;
    vector<Configuration*>* cfgS = ik.get_inv_kinematics(new SixDPos({0,0,0,0,0,0}));
    for (Configuration* cfg : *cfgS) {
        std::for_each(cfg->get_configuration().begin(), cfg->get_configuration().end(), [](double j) {cout << j << " ";});
        cout << endl;
    }

}