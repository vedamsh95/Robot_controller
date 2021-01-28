#ifndef SDRI_CTRL2019_TRAJECTORY_H
#define SDRI_CTRL2019_TRAJECTORY_H

#include <vector>
#include <iostream>
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
    
    int get_length();
    
    void append(Trajectory* endTrajectory);
    
    void insert(Configuration* config, int index);
    
    void set_configuration(Configuration* config, int index);
    
    void clear();
    
    void startAt(int index);

};


#endif //SDRI_CTRL2019_TRAJECTORY_H
