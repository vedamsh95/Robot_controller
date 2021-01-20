#ifndef SDRI_CTRL2019_TRAJECTORY_H
#define SDRI_CTRL2019_TRAJECTORY_H

#include <vector>
#include "Configuration.h"

using namespace std;

class Trajectory {
private:
    vector<Configuration*> m_configs;
public:
    Trajectory();

    Configuration* operator[](size_t index);


    Configuration* get_configuration(size_t index);

    vector<Configuration*>* get_all_configuration();

    void operator=(Trajectory& copy);


    void add_configuration(Configuration* config);

    void set_trajectory(vector<Configuration*> _trajectory);

    double Trajectory::max_vel_profile(double start_pos, double joint_distance, double joint_t_f, double joint_t_m,
                                       double t, double joint_t_pos, double a_max, double joint_v_max );

    double Trajectory::trapezoidal_profile(double joint_startpos, double joint_distance, double joint_endpos,
                                           double joint_t_f, double t, double joint_t_pos, double a_max, double joint_v_max);
};


#endif //SDRI_CTRL2019_TRAJECTORY_H
