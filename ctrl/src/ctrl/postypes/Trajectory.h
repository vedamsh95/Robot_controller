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
    
    /**
     *@return number of elements in trajectory
     */
    int get_length();
    
    /**
     * Appends endTrajectory to given trajectory
     *@param endTrajectory  trajectory to append
     */
    void append(Trajectory* endTrajectory);
    
    /**
     * Inserts configuration at index
     *@param config {@ref Configuration} configuration to insert
     *@param index at which it is inserted
     */
    void insert(Configuration* config, int index);
    /**
     * Inserts trajectory at index
     *@param trajectory {@ref trajectory} trajectory to insert
     *@param index at which it is inserted
     */
    void insert(Trajectory* trajectory, int index);
    
    /**
     * Sets configuration at given index
     *@param config {@ref Configuration}  that should be set
     *@param index where to set it
     */
    void set_configuration(Configuration* config, int index);
    
    /**
     * Clears the trajectory.
     */
    void clear();
    
    /**
     * Crops trajectory so it starts at given index.
     *@param index where trajectory should starrt
     */
    void startAt(int index);
    
    /**
     * Returns last element of trajectory.
     *@return {@ref Configuration} last element of trajectory.
     */
    Configuration* get_last();

};


#endif //SDRI_CTRL2019_TRAJECTORY_H
