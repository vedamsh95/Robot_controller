#include"IVMovement.h"

IVMovement::IVMovement()
{
    invK = new InvKinematics();
    fwK = new FwKinematics();
    trajectory = new Trajectory();
    robot = &Robot::getInstance();
    loopVector = new std::vector<std::vector<SixDPos*>>();
}

IVMovement::~IVMovement()
{
}

Trajectory * IVMovement::getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg, std::vector<std::vector<SixDPos*>>* loopPoints)
{
    loopVector = loopPoints;
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

        //check for elbow singularity.
        while(elbowSingularity(t)){
            t = esCalculation(t);
        }
        
        //calculate inverse kinematic for the position t.
        configs = invK->get_inv_kinematics(t, true);

        if (configs->size() > 0){
            correctConfig =  GetClosestConfiguration(configs, prevConfig);
            trajectory->add_configuration(correctConfig);
            
            if(overheadSingularity(t)){
                oSingularity = true;
                osLength++;
            }

            if(!overheadSingularity(t)&& oSingularity){
                osLength++;
                osInterpolation(trajectory, osLength, _positions);
                oSingularity = false;
                osLength = 0;
            }
            
            if(wristSingularity(trajectory->get_last())){
                wSingularity = true;
                wsLength++;
            }
            
            if(!wristSingularity(trajectory->get_last())&& wSingularity){
                wsLength++;
                wsInterpolation(trajectory, wsLength);
                wSingularity = false;
                wsLength = 0;
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
            trajectory->add_configuration(start_cfg);
            break;
        }
    }
    
    //Check if joint velocitys are in range and adjust them by adding points if necessary.
    CheckVelocities(trajectory, _positions);
    
    //crop trajectory because first Configuration* is current configuration of robot.
    trajectory->startAt(1);
    
    return trajectory;
}

Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig)
{
    int NbConfigs = _configs->size();
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
    int NbExceptions = 0;
    bool exception;
    
    //Check joint velocities that are too high due to distance between points and
    //add points to reduce the velocity.
    //(the difference inbetween consecutive joint configuratinos is higher than
    //the limits for the joint but lower than a certain threshold)
    for(int i = 1; i < _trajectory->get_length()+NbExceptions; i++)
    {
        exception = false;
        for(int j = 0; j < 6; j++)
        {
            if( std::abs
               (((*_trajectory->get_configuration(i-NbExceptions))[j] - ((*_trajectory->get_configuration(i-1-NbExceptions))[j]))
                         / robot->time_interval) > robot->velocities[j]){
                if (getMaxDiff(_trajectory->get_configuration(i-NbExceptions),
                               _trajectory->get_configuration(i-1-NbExceptions)) < 0.5*M_PI){
                    exception = true;
                }
            }
        }
        if(exception){
            if(Interpolate(_trajectory, _positions, i-NbExceptions))
                NbExceptions++;
            if(NbExceptions > 1000) break;
        }
        
    }

    if(NbExceptions > 0){
        cout << "Too high velocities detected, corresponding points changed." << endl;
    }
    
    //detect "flipping" of joints and interpolate configurations.
    NbExceptions = 0;
    std::vector<SixDPos*> temp;
    std::vector<int> LoopSize;
    std::vector<int> LoopStart;
    int actSize = 0;
    
    for(int i = 1; i < _trajectory->get_length()+NbExceptions; i++)
    {
        exception = false;
        for(int j = 0; j < 6; j++)
        {
            if(std::abs
               (((*_trajectory->get_configuration(i-NbExceptions))[j] - ((*_trajectory->get_configuration(i-1-NbExceptions))[j]))
                         / robot->time_interval) > robot->velocities[j]){
                exception = true;
            }
        }
        if(exception){
            //store values of start and size of each flipping loop.
            if(JointInterpolate(trajectory, i-NbExceptions)){
                if(LoopStart.size()==0){
                    LoopStart.push_back(i-NbExceptions);
                }
                else if(i-NbExceptions - (LoopStart.back()+actSize) > 3){
                    LoopSize.push_back(actSize+1);
                    actSize = 0;
                    LoopStart.push_back(i-NbExceptions);
                }
                else actSize++;
                
                NbExceptions++;
                if(NbExceptions > 1000) break;
            }
        }
    }
    LoopSize.push_back(actSize+1);

    getLoopPoints(LoopStart, LoopSize);
    
    if(NbExceptions > 0){
        cout << "Flipping of joints detected, configurations interpolated" << endl;
    }
}

/*
 Interpolation functions.
 */
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

    //check for elbow singularity.
    while(elbowSingularity(PosC)){
        PosC = esCalculation(PosC);
    }
    
    vector<Configuration*>* configs = invK->get_inv_kinematics(PosC, true );
    
    if(configs->size()>0)
    {
        Configuration* correctConfig = GetClosestConfiguration(configs, trajectory->get_configuration(index-1));
        
        //check for overhead singularity.
        if(overheadSingularity(PosC)){
            correctConfig = osInterpolation(trajectory->get_configuration(index-1),
                                  correctConfig,
                                  trajectory->get_configuration(index), PosC);
        }
        
        
        //check for wrist singularity.
        if(wristSingularity(correctConfig)){
            correctConfig = wsInterpolation(trajectory->get_configuration(index-1),
                                  correctConfig,
                                  trajectory->get_configuration(index));
        }
        
        //only add point if distance inbetween configurations is reduced by interpolation.
        if( distance(correctConfig, trajectory->get_configuration(index-1))
           < distance(trajectory->get_configuration(index),
                      trajectory->get_configuration(index-1))){
            _positions->insert(_positions->begin()+index, PosC);
            trajectory->insert(correctConfig, index);
            return true;
        }
        else return false;
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

/*
 Calculation of values at singularities.
 */

Configuration* IVMovement::wsInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig)
{
    Trajectory* A = new Trajectory();
    
    A->add_configuration(startConfig);
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    wsInterpolation(A, 2);
    
    return A->get_configuration(1);
}

void IVMovement::wsInterpolation(Trajectory* trajectory, int length)
{
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

Configuration* IVMovement::osInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig, SixDPos* pos)
{
    Trajectory* A = new Trajectory();
    vector<SixDPos*>* positions = new vector<SixDPos*>();
    
    A->add_configuration(startConfig);
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    positions->push_back(pos);
    positions->push_back(pos);
    
    osInterpolation(A, 2, positions);
    
    return A->get_configuration(1);
}

void IVMovement::osInterpolation(Trajectory* trajectory, int length, vector<SixDPos*>* _positions)
{
    vector<SixDPos*>* configs = new vector<SixDPos*>();
    int traSize = trajectory->get_length();
    Configuration* startConfig = trajectory->get_configuration(traSize-length-1);
    Configuration* endConfig = trajectory->get_configuration(traSize-1);
    std::array<double, 3> newConfig {};
    
    for(int i = 0; i < length; i++){
        Configuration* actConfig = trajectory->get_configuration(traSize-length+i);
        newConfig[0] = (*startConfig)[0] + (i+1)/(double)(length)*((*endConfig)[0]-(*startConfig)[0]);
        newConfig[1] = (*actConfig)[1];
        newConfig[2] = (*actConfig)[2];

        vector<Configuration*>* configs = invK->get_inv_kinematics(_positions->at(traSize-length+i), &newConfig);
        Configuration* correctConfig =  GetClosestConfiguration(configs, trajectory->get_configuration(traSize-length+i-1));
        
    trajectory->set_configuration(correctConfig, traSize-length+i);
    }
    //cout << "Theta1 configurations at overhead singularities interpolated. " << endl;
}

SixDPos* IVMovement::esCalculation(SixDPos* _pos)
{
    return new SixDPos(_pos->get_X()-(invK->sign(_pos->get_X())*0.05),
                    _pos->get_Y()-(invK->sign(_pos->get_Y())*0.05),
                    _pos->get_Z(),
                    _pos->get_A(),
                    _pos->get_B(),
                    _pos->get_C());
}



/*
 Conditions for singularities.
 */
bool IVMovement::wristSingularity(Configuration* _config)
{
    if(std::abs((*_config)[4])< SINGULARITY_MARGIN) return true;
    else return false;
}

bool IVMovement::overheadSingularity(SixDPos* _pos)
{
    std::array<double, 3> WCP = IVKinPos::getWristCenterPoint(_pos);
    if(std::abs(WCP.at(0))< SINGULARITY_MARGIN &&
       std::abs(WCP.at(1))< SINGULARITY_MARGIN){
        return true;
    }
    else return false;
}

bool IVMovement::elbowSingularity(SixDPos* _pos)
{
    std::array<double, 3> WCP = IVKinPos::getWristCenterPoint(_pos);
    
    double d1 = sqrt(pow(WCP.at(0),2)+pow(WCP.at(1),2));
    double d2 = sqrt(pow(robot->b, 2)+pow(robot->o, 2));
    
    double Px = abs(d1-robot->m);
    double Py = WCP.at(2) - robot->n;
    
    if(std::abs(pow((robot->a + d2), 2) - (Px*Px + Py*Py)) < SINGULARITY_MARGIN)
        return true;
    else return false;
        
}


/*
 Storage of interpolated values for visualization.
 */
void IVMovement::getLoopPoints(std::vector<int> LoopStart, std::vector<int> LoopSize)
{
    std::vector<SixDPos*> temp;
    
    for(int i=0; i < LoopStart.size(); i++)
    {
        //adjust the boundaries of loop points to draw the loop correctly.
        if (LoopStart.at(i)>0
            && LoopStart.at(i)+LoopSize.at(i) < trajectory->get_length()){
            LoopStart.at(i) = LoopStart.at(i)-1;
            LoopSize.at(i) = LoopSize.at(i)+2;
        }
        else if(LoopStart.at(i)>0){
            LoopStart.at(i) = LoopStart.at(i)-1;
            LoopSize.at(i) = LoopSize.at(i)+1;
        }
        else if(LoopStart.at(i)+LoopSize.at(i) < trajectory->get_length()){
            LoopSize.at(i) = LoopSize.at(i)+1;
        }
        
        for(int j= LoopStart.at(i); j < (LoopStart.at(i)+LoopSize.at(i)); j++)
        {
            temp.push_back(fwK->get_fw_kinematics(trajectory->get_configuration(j)));
        }
        loopVector->push_back(temp);
        temp.clear();
    }
}

/*
 General calculations.
 */

double IVMovement::getMaxDiff(Configuration* ConfigA, Configuration* ConfigB)
{
    double maxDist = 0;
    for(int i= 0; i < NUM_JOINTS; i++){
        if(std::abs((*ConfigA)[i]-(*ConfigB)[i]) > maxDist){
            maxDist = std::abs((*ConfigA)[i]-(*ConfigB)[i]);
        }
    }
    return maxDist;
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
