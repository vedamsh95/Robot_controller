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

void Trajectory::append(Trajectory* endTrajectory)
{
    for(int i= 0; i< endTrajectory->get_length(); i++)
    {
        this->m_configs.push_back(endTrajectory->get_configuration(i));
    }
}

void Trajectory::insert(Configuration* config, int index)
{
    this->m_configs.insert(m_configs.begin()+ index, config);
}

void Trajectory::insert(Trajectory* _trajectory, int index)
{
    this->m_configs.insert(m_configs.begin()+ index, _trajectory->m_configs.begin(), _trajectory->m_configs.end());
}

void Trajectory::clear()
{
    this->m_configs.clear();
}

void Trajectory::set_configuration(Configuration *config, int index)
{
    this->m_configs.at(index) = config;
}

void Trajectory::startAt(int index)
{
    vector<Configuration*> B;
    for(int i = index; i < this->m_configs.size(); i++)
    {
        B.push_back(this->m_configs.at(i));
    }
    this->m_configs.clear();
    this->m_configs = B;
}

Configuration* Trajectory::get_last()
{
    return this->m_configs.back();
}
