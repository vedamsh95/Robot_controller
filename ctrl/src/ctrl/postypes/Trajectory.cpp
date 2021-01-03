#include "Trajectory.h"
#include "iostream"
Trajectory::Trajectory()
{

}


Configuration* Trajectory::operator[](size_t index)
{
    return this->m_configs[index];
}


vector<Configuration*>* Trajectory::get_all_configuration()
{
    return &(this->m_configs);
}


Configuration* Trajectory::get_configuration(size_t index)
{
    return this->m_configs[index];
}


void Trajectory::operator=(Trajectory& copy)
{
    this->set_trajectory( *(copy.get_all_configuration()) );
}


void Trajectory::set_trajectory(const vector<Configuration*> _configurations)
{
    this->m_configs.clear();
    this->m_configs = _configurations;
}

//function that returns position of a joint at time t
double Trajectory::max_vel_profile(double joint_startpos, double joint_distance, double joint_t_f, double joint_t_m,
                                   double t, double joint_t_pos, double a_max, double joint_v_max ) {
    double joint_t_c = joint_v_max / a_max;
    if (joint_t_c >= joint_t_m) {   // check if we need max_vel_profile
        //calculate joint1 position at time = t
        if (t < 0) {
            return joint_t_pos;
        } else if (t < joint_t_m) {
            joint_t_pos = joint_startpos + 2 * pow((t / joint_t_f), 2) * joint_distance;
            return joint_t_pos;
        } else if ((joint_t_m < t) && (t <= joint_t_f)) {
            joint_t_pos = joint_startpos + (-1 + 4 * (t / joint_t_f) - 2 * pow((t / joint_t_f), 2)) * joint_distance;
            return joint_t_pos;
        } else {
            return joint_t_pos;     // no change to position
        }
    }
}

double Trajectory::trapezoidal_profile(double joint_startpos, double joint_distance, double joint_endpos, double joint_t_m,
                                       double joint_t_f, double t, double joint_t_pos, double a_max, double joint_v_max) {
    double joint_t_c = joint_v_max / a_max;
    if (joint_t_c < joint_t_m) {   // check if we need max_vel_profile
        joint_t_f = joint_t_c + (abs(joint_distance) / joint_v_max);
        if (t < 0) {
            return joint_t_pos;
        } else if (t < joint_t_c ) {
            joint_t_pos = joint_startpos + 0.5 * a_max * pow(t, 2) * ( joint_distance/abs(joint_distance) );    // last part to get sign of distance
        } else if( (joint_t_c < t) && ( t <= (joint_t_f - joint_t_c) ) ) {
            joint_t_pos = joint_startpos + a_max * joint_t_c * (t - (joint_t_c /2) ) * ( joint_distance/abs(joint_distance) );
        } else if( ( (joint_t_f - joint_t_c) < t ) && (t <= joint_t_f) ){
            joint_t_pos = joint_endpos - 0.5 * a_max * pow(joint_t_f - t, 2) * ( joint_distance/abs(joint_distance) );
        }
    }
    return joint_t_pos;
}
