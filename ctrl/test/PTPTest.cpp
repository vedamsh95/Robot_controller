//
// Created by Tim on 24.02.2021.
//

#include <Trajectory.h>
#include "gtest/gtest.h"
#include "fw_kinematics.h"
#include "inverse_kinematics.h"
#include "ptp.h"

TEST(PTP, sync)
{
    Ptp planner;

    Trajectory* t = planner.get_ptp_trajectoy(
            new Configuration({0,0,0,0,0,0}),
            new Configuration({1,1,1,1,1,1}));


    for (Configuration* cfg : *(t->get_all_configuration())) {
        std::for_each(cfg->get_configuration().begin(), cfg->get_configuration().end(), [](double j) {cout << j << " ";});
        cout << endl;
    }
}

TEST(PTP, async)
{


}