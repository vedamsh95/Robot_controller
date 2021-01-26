#include"IVMovement.h"

IVMovement::IVMovement()
{
    invK = new InvKinematics();
    trajectory = new Trajectory();
    robot = &Robot::getInstance();
}

IVMovement::~IVMovement()
{
}

Trajectory * IVMovement::getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg)
{
    Configuration* prevConfig = start_cfg;
    Configuration* correctConfig;
    vector<Configuration*>* configs;
    Trajectory* singTrajectory = new Trajectory(); 
    SixDPos* t;
    bool singularity = false;
    int SingStart;
    int SingLength = 0;

    for (int i = 0; i< _positions->size(); i++)
    {
        t = _positions->at(i);
        //TODO: check for overhead and elbow singularity.
        //(before IVKinematics is calculated!)
        configs = invK->get_inv_kinematics(t, true);
        
        if (configs->size() > 0){
            correctConfig =  GetClosestConfiguration(configs, prevConfig);
        
            //Wrist singularity.
            if((*correctConfig)[4] == 0 && i != (_positions->size()-1)){
                //cout << "Configuration " << i << " is wrist singularity!" << endl;
                if(!singularity){
                    singularity = true;
                    singTrajectory->add_configuration(correctConfig);
                }
                else singTrajectory->add_configuration(correctConfig);
            }
            else{
                if(singularity){
                    singTrajectory->add_configuration(correctConfig);
                    trajectory->append(wsInterpolation(prevConfig, singTrajectory));
                    singularity = false;
                    singTrajectory->clear();
                    prevConfig = correctConfig;
                }
                else{
                    prevConfig = correctConfig;
                    trajectory->add_configuration(correctConfig);
                }
            }
        }
        else{
            cout << "Trajectory can not be calculated!" << endl; 
            trajectory->clear();
            trajectory->add_configuration(start_cfg);
            break;
        }
    }
    
    //Check if joint velocitys are in range and adjust them by adding points if necessary.
    CheckVelocities(trajectory, _positions);
    
    return trajectory;
}

Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig) {
    vector<Configuration*>::size_type NbConfigs = _configs->size();
    double minDist = 1000;
    int minConfig = 0;
    for (int i = 0; i < NbConfigs; i++) {
        Configuration* ActConfig = _configs->at(i);
        double actDist = 0;
        for (int j = 0; j < NUM_JOINTS; j++) {
            actDist += pow((*ActConfig)[j] - (*_prevConfig)[j], 2);
        }
        actDist = sqrt(actDist);

        if (actDist < minDist) {
            minDist = actDist;
            minConfig = i;
        }
    }
    return _configs->at(minConfig);

}

void IVMovement::CheckVelocities(Trajectory* _trajectory, vector<SixDPos*>* _positions)
{
    int NbExeptions = 0;
    cout << "Checking velocities" << endl;
    for(int i = 1; i < _trajectory->get_length()+NbExeptions; i++)
    {
        bool exeption = false;
        for(int j = 0; j < 6; j++)
        {
            if( std::abs
               (((*_trajectory->get_configuration(i-NbExeptions))[j] - ((*_trajectory->get_configuration(i-1-NbExeptions))[j]))
                         / robot->time_interval) > robot->velocities[j]){
                exeption = true;
            }
        }
        if(exeption){
            if(Interpolate(_trajectory, _positions, i-NbExeptions)){
                NbExeptions++;
            }
            if(NbExeptions > 500) break;
        }
    }

    if(NbExeptions > 0){
        cout << "Too high velocities detected, corresponding points changed." << endl;
    }
    else return;
    
}

bool IVMovement::Interpolate(Trajectory *trajectory, vector<SixDPos*>* _positions, int index)
{
    SixDPos* PosA = _positions->at(index-1);
    SixDPos* PosB = _positions->at(index);
    SixDPos* PosC = new SixDPos(PosA->get_X()+ 0.5*(PosB->get_X()-PosA->get_X()),
                               PosA->get_Y()+ 0.5*(PosB->get_Y()-PosA->get_Y()),
                               PosA->get_Z()+ 0.5*(PosB->get_Z()-PosA->get_Z()),
                               PosA->get_A(),
                               PosA->get_B(),
                               PosA->get_C());

    
    Configuration* correctConfig;
    vector<Configuration*>* configs;
    configs = invK->get_inv_kinematics(PosC);
    correctConfig =  GetClosestConfiguration(configs, trajectory->get_configuration(index-1));
    
    //check for wrist singularity.
    if((*correctConfig)[4] == 0){
        correctConfig = wsInterpolation(trajectory->get_configuration(index-1),
                              correctConfig,
                              trajectory->get_configuration(index));
    }
    
    if( distance(correctConfig, trajectory->get_configuration(index-1))
       < distance(trajectory->get_configuration(index), trajectory->get_configuration(index-1))){
        _positions->insert(_positions->begin()+index, PosC);
        trajectory->insert(correctConfig, index);
        return true;
    }
    else return false;
    
}

Configuration* IVMovement::wsInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig)
{
    Trajectory* A = new Trajectory();
    Trajectory* C = new Trajectory();
    
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    C = wsInterpolation(startConfig, A);
    
    return C->get_configuration(0);
}

Trajectory* IVMovement::wsInterpolation(Configuration* startConfig, Trajectory* trajectory)
{
    Trajectory* configs = new Trajectory();
    int length = trajectory->get_length();
    Configuration* endConfig = trajectory->get_configuration(length-1);
    std::array<double, NUM_JOINTS> newConfig {};
    
    for(int i = 0; i < length; i++){
        Configuration* actConfig = trajectory->get_configuration(i);
        for(int j= 0; j < 3; j++){
            newConfig[j] = (*actConfig)[j];
        }
        newConfig[3] = (*startConfig)[3] + (i+1)/(double)(length)*((*endConfig)[3]-(*startConfig)[3]);
        newConfig[4] = (*actConfig)[4];
        newConfig[5] = (*startConfig)[5] + (i+1)/(double)(length)*((*endConfig)[5]-(*startConfig)[5]);
        
        configs->add_configuration(new Configuration(newConfig));
    }
    //cout << "Wrist configurations at singularities interpolated. " << endl;
    return configs;
}

double IVMovement::distance(Configuration* A, Configuration* B)
{
    double distance = 0;
    for(int i=0; i < NUM_JOINTS; i++)
    {
        distance += pow(((*A)[i]-(*B)[i]), 2);
    }
    return sqrt(distance);
}




