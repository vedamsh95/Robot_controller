#include "Trajectory.h"
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
void Trajectory::add_configuration(Configuration* config)
{
	this->m_configs.push_back(config);
}

int Trajectory::get_length()
{
    return this->m_configs.size();
}

void Trajectory::append(Trajectory *endTrajectory)
{
    for(int i= 0; i< endTrajectory->get_length(); i++)
    {
        this->m_configs.push_back(endTrajectory->get_configuration(i));
    }
}

