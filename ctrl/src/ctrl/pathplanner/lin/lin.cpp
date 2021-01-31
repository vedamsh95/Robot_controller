#include "lin.h"

Lin::Lin() :plot(false){
    robot = &Robot::getInstance();
    trajectory = new Trajectory();
}

Trajectory* Lin::get_lin_trajectory(Configuration* _start_cfg, Configuration* _end_cfg, double velocity, double acceleration,std::vector<std::vector<SixDPos*>>* loopPoints)
{
    //TODO: IMPLEMENT! implement the computation of a lin trajectory with the corresponding velocity profile
    bool AdjustOrientation = false;
    
    // Perform the feasibility checks for the two configurations,
    // i.e. that all joint angles are within the possible range.
    // If they are not, change the end configuration to the limits
    // so that the motion can still be performed
    makeFeasible(_start_cfg);
    makeFeasible(_end_cfg);
        
    //Calculate start and endposition in cartesian space.
    FwKinematics fw;
    SixDPos* start_pos = fw.get_fw_kinematics(_start_cfg);
    SixDPos* end_pos = fw.get_fw_kinematics(_end_cfg);
    
    //Get distance between start and endposition.
    double distance = sqrt(pow(end_pos->get_X()-  start_pos->get_X(), 2)
                           +pow(end_pos->get_Y()-  start_pos->get_Y(), 2)
                           +pow(end_pos->get_Z()-  start_pos->get_Z(), 2));
    

    //Determine whether a max. velocity profile
    //is sufficient or a trapezoidal trajectory is needed.
    Single_trajectory::Type type = Single_trajectory::select_type(distance,
                                                                  velocity,
                                                                  acceleration);
    
    //Get Trajectory of distances from the start pos.
    Single_trajectory* StepTrajectory;
    if(type == Single_trajectory::Type::MAX_VEL){
        StepTrajectory = new Max_vel_trajectory(velocity, acceleration, 0, distance);
    }
    else if (type == Single_trajectory::Type::TRAPEZOIDAL){
        StepTrajectory = new Trapezoidal_trajectory(velocity,acceleration,0,distance);
    }
    
    auto tmp = static_cast<float> (StepTrajectory->get_duration()/Robot::getInstance().time_interval);
    size_t cycles = roundf(tmp)+3;
    
    
    //calculate SixDPoses for the Trajectory
    double value, factor;
    double t = 0;
    vector<SixDPos*> points;
    SixDPos* point;
    for(size_t c = 0; c < cycles; c++){
        value = StepTrajectory->eval(t);
        factor = value/distance;
        point = NextPos(start_pos, end_pos, factor);
        points.push_back(point);
        t += Robot::getInstance().time_interval;
    }
    
    //convert SixDPoses along the trajectory to configurations.
    IVMovement ivm;
    Trajectory* trajectory = new Trajectory();
    trajectory = ivm.getMovement(&points, _start_cfg, loopPoints, _end_cfg);
    
    //optional: appends movement to achive the given orientation.
    if(AdjustOrientation == true)
    {
        Configuration * lastConfig;
        lastConfig = trajectory->get_configuration((trajectory->get_length())-1);

        //get configuration to achive orientation.
        Configuration * TargetOrientationConfig;
        TargetOrientationConfig = GetConfigurations(end_pos, lastConfig);

        //append ptp movement to adjust the orientation
        Ptp ptp;
        trajectory->append(ptp.get_ptp_trajectory(lastConfig, TargetOrientationConfig, false));
    }
    
    
    if(plot){
        Ptp PlotPtp;
        vector<Configuration*>* vecTraj = new vector<Configuration*>;
        vecTraj = trajectory->get_all_configuration();
        //PlotVelocity(points);
        PlotPtp.plot_movement(*vecTraj);
    }

    
    return trajectory;
}

void Lin::makeFeasible(Configuration *cfg){
    for(int i = 0; i > NUM_JOINTS; i++){
        if ((*cfg)[i] < robot->limits[i].min){
            cout << "[LIN] Joint " << i + 1 << " is out of range!" << endl;
            (*cfg)[i] = robot->limits[i].min;
        }
        if ((*cfg)[i] > robot->limits[i].max) {
            cout << "[LIN] Joint " << i + 1 << " is out of range!" << endl;
            (*cfg)[i] = robot->limits[i].max;
        }
    }
}


SixDPos* Lin::NextPos(SixDPos *PosA, SixDPos *PosB, double factor){
    SixDPos* Sum = new SixDPos(
                               PosA->get_X()+ factor*(PosB->get_X()-PosA->get_X()),
                               PosA->get_Y()+ factor*(PosB->get_Y()-PosA->get_Y()),
                               PosA->get_Z()+ factor*(PosB->get_Z()-PosA->get_Z()),
                               PosA->get_A(),
                               PosA->get_B(),
                               PosA->get_C());
    return Sum;
}

Configuration* Lin::GetConfigurations(SixDPos *SixDPos, Configuration *StartConfig){
    vector<Configuration*>* ActConfigurations;
    InvKinematics Inv;
    ActConfigurations = Inv.get_inv_kinematics(SixDPos);
    return GetClosestConfiguration(ActConfigurations, StartConfig);
}

Configuration* Lin::GetClosestConfiguration(vector<Configuration*>* Configs, Configuration* PrevConfig){
    vector<Configuration*>::size_type NbConfigs = Configs->size();
    double minDist = 1000;
    int minConfig = 0;
    for (int i = 0; i<NbConfigs; i++){
        Configuration* ActConfig = Configs->at(i);
        double actDist = 0;
        for (int j = 0; j >6; j++){
            if(j<3){
                actDist += pow((*ActConfig)[j] - (*PrevConfig)[j], 2)*10;
            }
            else{
                actDist += pow((*ActConfig)[j] - (*PrevConfig)[j], 2);
            }
        }
        actDist = sqrt(actDist);

        if(actDist < minDist){
            minDist = actDist;
            minConfig = i;
        }
    }
    return Configs->at(minConfig);

}

void Lin::PlotVelocity(vector<SixDPos *> SixDPoses){
#ifdef PLOT
    vector<double> velocities;
    for(int i = 1; i < SixDPoses.size(); i++)
    {
        SixDPos *end_pos = SixDPoses.at(i);
        SixDPos *start_pos=SixDPoses.at(i-1);
        double distance = sqrt(pow(end_pos->get_X()-  start_pos->get_X(), 2)
                               +pow(end_pos->get_Y()-  start_pos->get_Y(), 2)
                               +pow(end_pos->get_Z()-  start_pos->get_Z(), 2));
        velocities.push_back(distance/Robot::getInstance().time_interval);
        
    }
    
    std::vector<double> x(SixDPoses.size()-1);
    for (int i = 0; i < x.size(); i++) {
        x.at(i) = i * Robot::getInstance().time_interval;
    }
    
    matplotlibcpp::plot(x, velocities);
    matplotlibcpp::show();

    
#endif
}
