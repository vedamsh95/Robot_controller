//
// Created by Tim on 24.02.2021.
//

#include <pathplanner/lin/lin.h>
#include "gtest/gtest.h"
#include "fw_kinematics.h"
#include "inverse_kinematics.h"

TEST(LinMove, direct)
{
    Lin planner;

    Trajectory* t = planner.get_lin_trajectoy(
            new Configuration({0,0,0,0,0,0}),
            new Configuration({1,1,1,1,1,1}));


    for (Configuration* cfg : *(t->get_all_configuration())) {
        std::for_each(cfg->get_configuration().begin(), cfg->get_configuration().end(), [](double j) {cout << j << " ";});
        cout << endl;
    }}
