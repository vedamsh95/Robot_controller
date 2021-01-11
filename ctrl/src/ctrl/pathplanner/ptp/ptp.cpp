#include "ptp.h"


Trajectory* Ptp::get_ptp_trajectory(Configuration* _start_cfg, Configuration* _end_cfg)
{
    //TODO: IMPLEMENT! implement the computation of a ptp trajectory with the corresponding velocity profile
    Trajectory* trajectory = new Trajectory();
    //Dummy trajectory
    vector<Configuration *> config;
    array<double, 6> start_cfg = _start_cfg->get_configuration();
    array<double, 6> end_cfg = _end_cfg->get_configuration();


    for(float i; i<=1; i+=0.1) {
        config.push_back(new Configuration(
                {start_cfg[0] + (end_cfg[0] - start_cfg[0]) * (i),
                 start_cfg[1] + (end_cfg[1] - start_cfg[1]) * (i),
                 start_cfg[2] + (end_cfg[2] - start_cfg[2]) * (i),
                 start_cfg[3] + (end_cfg[3] - start_cfg[3]) * (i),
                 start_cfg[4] + (end_cfg[4] - start_cfg[4]) * (i),
                 start_cfg[5] + (end_cfg[5] - start_cfg[5]) * (i)})

                );
    }


    trajectory->set_trajectory(config);

    return trajectory;
}

