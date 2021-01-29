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
    SixDPos* t;
    bool wSingularity = false;
    bool oSingularity = false;
    int wsLength = 0;
    int osLength = 0;
    trajectory->add_configuration(start_cfg);


    for (int i = 1; i< _positions->size(); i++)
    {
        t = _positions->at(i);
        //TODO: check for elbow singularity.
        //(before IVKinematics is calculated!)
        while(elbowSingularity(t)){
            cout << "Elbow Singularity" << endl;
            t = new SixDPos(t->get_X()-(invK->sign(t->get_X())*0.05),
                            t->get_Y()-(invK->sign(t->get_Y())*0.05),
                            t->get_Z(),
                            t->get_A(),
                            t->get_B(),
                            t->get_C());
        }
        
        configs = invK->get_inv_kinematics(t, true);

        if (configs->size() > 0){
            correctConfig =  GetClosestConfiguration(configs, prevConfig);
            trajectory->add_configuration(correctConfig);
            
            if(wristSingularity(correctConfig)){
                wSingularity = true;
                wsLength++;
            }
            if(overheadSingularity(t)){
                oSingularity = true;
                osLength++;
            }

            if(!wristSingularity(correctConfig)&& wSingularity){
                wsLength++;
                wsInterpolation(trajectory, wsLength);
                wSingularity = false;
                wsLength = 0;
            }

            if(!overheadSingularity(t)&& oSingularity){
                osLength++;
                osInterpolation(trajectory, osLength);
                oSingularity = false;
                osLength = 0;
            }

            if(!wristSingularity(correctConfig)&& !wSingularity &&
               !overheadSingularity(t)&& !oSingularity){
                prevConfig = correctConfig;
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
    //CheckVelocities(trajectory, _positions);
    CheckJointVelocities(trajectory);
    
    //crop trajectory because first Configuration* is current configuration of robot.
    trajectory->startAt(1);
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

void IVMovement::CheckJointVelocities(Trajectory* _trajectory)
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
            if(JointInterpolate(trajectory, i-NbExeptions)){
                NbExeptions++;
            }
            if(NbExeptions > 500) break;
        }
    }

    if(NbExeptions > 0){
        cout << "Too high velocities detected, corresponding points changed." << endl;
    }
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
    configs = invK->get_inv_kinematics(PosC, true );
    correctConfig =  GetClosestConfiguration(configs, trajectory->get_configuration(index-1));
    
    //check for wrist singularity.
    if(wristSingularity(correctConfig)){
        correctConfig = wsInterpolation(trajectory->get_configuration(index-1),
                              correctConfig,
                              trajectory->get_configuration(index));
    }
    
    //check for overhead singularity.
    if(overheadSingularity(PosC)){
        correctConfig = osInterpolation(trajectory->get_configuration(index-1),
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

bool IVMovement::JointInterpolate(Trajectory* trajectory, int index)
{
    Configuration* ConfigA = trajectory->get_configuration(index-1);
    Configuration* ConfigB = trajectory->get_configuration(index);
    std::array<double, NUM_JOINTS> newConfig {};

    for(int j= 0; j < NUM_JOINTS; j++){
        newConfig[j] = (*ConfigA)[j] + 0.5* ((*ConfigB)[j]- (*ConfigA)[j]);
    }
    
    Configuration* ConfigC = new Configuration(newConfig);
    
    if( distance(ConfigC, ConfigB) < distance(ConfigA, ConfigB)){
        trajectory->insert(ConfigC, index);
        return true;
    }
    else return false;
}


Configuration* IVMovement::wsInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig)
{
    Trajectory* A = new Trajectory();
    Trajectory* C = new Trajectory();
    
    A->add_configuration(startConfig);
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    wsInterpolation(A, 2);
    
    return A->get_configuration(1);
}

Configuration* IVMovement::osInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig)
{
    Trajectory* A = new Trajectory();
    Trajectory* C = new Trajectory();
    
    A->add_configuration(startConfig);
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    osInterpolation(A, 2);
    
    return A->get_configuration(1);
}

void IVMovement::wsInterpolation(Trajectory* trajectory, int length)
{
    Trajectory* configs = new Trajectory();
    int traSize = trajectory->get_length();
    Configuration* startConfig = trajectory->get_configuration(traSize-length-1);
    Configuration* endConfig = trajectory->get_configuration(traSize-1);
    std::array<double, NUM_JOINTS> newConfig {};

    for(int i = 0; i < length; i++){
        Configuration* actConfig = trajectory->get_configuration(traSize-length+i);
        for(int j= 0; j < 3; j++){
            newConfig[j] = (*actConfig)[j];
        }
        newConfig[3] = (*startConfig)[3] + (i+1)/(double)(length)*((*endConfig)[3]-(*startConfig)[3]);
        newConfig[4] = (*actConfig)[4];
        newConfig[5] = (*startConfig)[5] + (i+1)/(double)(length)*((*endConfig)[5]-(*startConfig)[5]);

        trajectory->set_configuration(new Configuration(newConfig), traSize-length+i);
    }
    //cout << "Wrist configurations at singularities interpolated. " << endl
}


void IVMovement::osInterpolation(Trajectory* trajectory, int length){
    Trajectory* configs = new Trajectory();
    int traSize = trajectory->get_length();
    Configuration* startConfig = trajectory->get_configuration(traSize-length-1);
    Configuration* endConfig = trajectory->get_configuration(traSize-1);
    std::array<double, NUM_JOINTS> newConfig {};
    
    for(int i = 0; i < length; i++){
        Configuration* actConfig = trajectory->get_configuration(traSize-length+i);
        newConfig[0] = (*startConfig)[0] + (i+1)/(double)(length)*((*endConfig)[0]-(*startConfig)[0]);
        for(int j= 1; j < NUM_JOINTS; j++){
            newConfig[j] = (*actConfig)[j];
        }
        trajectory->set_configuration(new Configuration(newConfig), traSize-length+i);
    }
    cout << "Theta1 configurations at overhead singularities interpolated. " << endl;
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


bool IVMovement::wristSingularity(Configuration* _config)
{
    if(std::abs((*_config)[4])< SINGULARITY_MARGIN) return true;
    else return false;
}

bool IVMovement::overheadSingularity(SixDPos* _pos)
{
    if(std::abs(_pos->get_X())< SINGULARITY_MARGIN &&
       std::abs(_pos->get_Y())< SINGULARITY_MARGIN){
        return true;
    }
    else return false;
}

bool IVMovement::elbowSingularity(SixDPos* _pos)
{
    double d1 = sqrt(pow(_pos->get_X(),2)+pow(_pos->get_Y(),2));
    double d2 = sqrt(pow(robot->b, 2)+pow(robot->o, 2));
    
    double Px = abs(d1-robot->m);
    double Py = _pos->get_Z() - robot->n;
    
    if(std::abs(pow((robot->a + d2), 2) - (Px*Px + Py*Py)) < SINGULARITY_MARGIN)
        return true;
    else return false;
        
}

