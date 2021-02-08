#include "lin.h"

Lin::Lin() :plot(false){
    robot = &Robot::getInstance();
    trajectory = new Trajectory();
    invK = new InvKinematics();
    fwK = new FwKinematics();
    ivM = new IVMovement();
    ptp = new Ptp();
}

Trajectory* Lin::get_lin_trajectory(Configuration* _start_cfg, Configuration* _end_cfg, double velocity, double acceleration,std::vector<std::vector<SixDPos*>>* loopPoints)
{
    //choose weather a movement should be appended to achive the given
    //orientation at the last point.
    bool AdjustOrientation = false;
    
    // Perform the feasibility checks for the two configurations,
    // i.e. that all joint angles are within the possible range.
    // If they are not, change the end configuration to the limits
    // so that the motion can still be performed
    makeFeasible(_start_cfg);
    makeFeasible(_end_cfg);
        
    //Calculate start and endposition in cartesian space.
    SixDPos* start_pos = fwK->get_fw_kinematics(_start_cfg);
    SixDPos* end_pos = fwK->get_fw_kinematics(_end_cfg);
    
    //Get distance between start and endposition.
    double distance = sqrt(pow(end_pos->get_X()-  start_pos->get_X(), 2)
                           +pow(end_pos->get_Y()-  start_pos->get_Y(), 2)
                           +pow(end_pos->get_Z()-  start_pos->get_Z(), 2));
    

    //Determine whether a max. velocity profile
    //is sufficient or a trapezoidal trajectory is needed.
    Single_trajectory::Type type = Single_trajectory::select_type(distance,
                                                                  velocity,
                                                                  acceleration);
    
    //Get trajectory of distances from the start_pos.
    Single_trajectory* StepTrajectory;
    if(type == Single_trajectory::Type::MAX_VEL){
        StepTrajectory = new Max_vel_trajectory(velocity, acceleration, 0, distance);
    }
    else if (type == Single_trajectory::Type::TRAPEZOIDAL){
        StepTrajectory = new Trapezoidal_trajectory(velocity,acceleration,0,distance);
    }
    
    auto tmp = static_cast<float> (StepTrajectory->get_duration()/Robot::getInstance().time_interval);
    size_t cycles = roundf(tmp)+3;
    
    
    //calculate SixDPoses for trajectory.
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
    
    //store points for plotting before they are adjusted due to reduction
    //of joint velocities.
    vector<SixDPos*> unchangedPoints = points;
    
    //get closest possible end configuration to start configuration
    //to prevent unnecessary rotations. This configuration is used to
    //determine most effective configuration in overhead singularity.
    Configuration * bestEnd_cfg = GetConfigurations(points.back(), _start_cfg);
    
    //convert SixDPoses along the trajectory to configurations.
    Trajectory* trajectory = ivM->getMovement(&points, _start_cfg, loopPoints, bestEnd_cfg);
    
    //optional: appends movement to achive the given orientation.
    if(AdjustOrientation)
    {
        Configuration * lastConfig;
        lastConfig = trajectory->get_configuration((trajectory->get_length())-1);

        //get configuration to achive orientation.
        Configuration * TargetOrientationConfig;
        TargetOrientationConfig = GetConfigurations(end_pos, lastConfig);

        //append ptp movement to adjust the orientation
        trajectory->append(ptp->get_ptp_trajectory(lastConfig, TargetOrientationConfig, false));
    }
    
    
    if(plot){
        vector<Configuration*>* allConfigs = trajectory->get_all_configuration();        
        //detect if path got aborted.
        bool allzero = true;
        for(double x : allConfigs->back()->get_configuration()){
            if(x != 0) allzero = false;
        }
        if(allzero) allConfigs->pop_back();
        
        //plot 3D path and total velocity.
        Plot(unchangedPoints);
        
        //plot joint positions and velocity.
        ptp->plot_movement(*allConfigs);
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
    return new SixDPos(PosA->get_X()+ factor*(PosB->get_X()-PosA->get_X()),
                       PosA->get_Y()+ factor*(PosB->get_Y()-PosA->get_Y()),
                       PosA->get_Z()+ factor*(PosB->get_Z()-PosA->get_Z()),
                       PosA->get_A(),
                       PosA->get_B(),
                       PosA->get_C());
}

Configuration* Lin::GetConfigurations(SixDPos *SixDPos, Configuration *StartConfig)
{
    vector<Configuration*>* ActConfigurations = invK->get_inv_kinematics(SixDPos);
    if (ActConfigurations->size()>0){
        return ivM->GetClosestConfiguration(ActConfigurations, StartConfig, true);
    }
    else return nullptr;
}


void Lin::Plot(vector<SixDPos *> SixDPoses){
#ifdef PLOT
    
    //3D plot of trajectory from calculated points.
    std::vector<double> x, y, z;
    
    for (int i = 0; i < SixDPoses.size(); i++) {
        x.push_back(SixDPoses[i]->get_X());
        y.push_back(SixDPoses[i]->get_Y());
        z.push_back(SixDPoses[i]->get_Z());
    }
    

    matplotlibcpp::plot3(x, y, z);
    
    matplotlibcpp::title("Path of LIN movement");
    matplotlibcpp::xlabel("x");
    matplotlibcpp::ylabel("y");
    matplotlibcpp::set_zlabel("z");
    matplotlibcpp::xlim(-3, 3);
    matplotlibcpp::ylim(-3, 3);
    
    matplotlibcpp::legend();
    matplotlibcpp::show();
    
    //plot of velcities derived from point distances. 
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
    
    std::vector<double> t(SixDPoses.size()-1);
    for (int i = 0; i < t.size(); i++) {
        t.at(i) = i * Robot::getInstance().time_interval;
    }
    
    matplotlibcpp::title("Total velocity");
    matplotlibcpp::plot(t, velocities);
    
    matplotlibcpp::show();

    
#endif
}
