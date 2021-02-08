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

Trajectory * IVMovement::getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg, std::vector<std::vector<SixDPos*>>* loopPoints, Configuration* end_cfg)
{
    loopVector = loopPoints;
    Configuration* prevConfig = start_cfg;
    Configuration* correctConfig;
    vector<Configuration*>* configs;
    SixDPos* t;
    
    bool pathCancelled = false;
    bool wSingularity = false;
    bool oSingularity = false;
    bool lastPoint = false;
    int wsLength = 0;
    int osLength = 0;
    
    //add first configuration in case it is needed for interpolation.
    trajectory->add_configuration(start_cfg);

    for (int i = 1; i< _positions->size(); i++)
    { 
        if(i == _positions->size()-1) lastPoint=true;
        t = _positions->at(i);

        //check for elbow singularity and adjust t if necessary.
        while(elbowSingularity(t)){
            t = esCalculation(t);
        }
        
        //calculate inverse kinematics for the position t.
        configs = invK->get_inv_kinematics(t);

        if (configs->size() > 0){
            //find configuration that is closest to previous configuration.
            correctConfig =  GetClosestConfiguration(configs, prevConfig);
            trajectory->add_configuration(correctConfig);
            
            //normal case.
            if(!wristSingularity(correctConfig)&& !wSingularity &&
               !overheadSingularity(t)&& !oSingularity){
                prevConfig = correctConfig;
            }
            
            //overhead singularity.
            if(overheadSingularity(t)){
                oSingularity = true;
                osLength++;
            }
            
            //point after overhead singularity.
            if(oSingularity && (!overheadSingularity(t) || lastPoint)){
                if(lastPoint) osLastConfig(correctConfig, t, osLength);
                //if end_cfg available the configuration after the singularity
                //should resemble the configuration at the end of the trajectory.
                else if(end_cfg){
                    osLength++;
                    correctConfig = GetClosestConfiguration(configs, end_cfg, correctConfig);
                    trajectory->set_configuration(correctConfig, trajectory->get_length()-1);
                    prevConfig = correctConfig;
                }
                else osLength++;
                osInterpolation(trajectory, osLength, _positions);
                oSingularity = false;
                osLength = 0;
            }
            
            //wrist singularity.
            if(wristSingularity(trajectory->get_last())){
                wSingularity = true;
                wsLength++;
            }
            
            //configuration after wrist singularity.
            if(wSingularity && (!wristSingularity(trajectory->get_last()) || lastPoint)){
                if(lastPoint) wsLastConfig(correctConfig, wsLength);
                else wsLength++;
                wsInterpolation(trajectory, wsLength);
                wSingularity = false;
                wsLength = 0;
            }
        }
        else{
            cout << "Trajectory can not be calculated!" << endl;
            pathCancelled = true;
            break;
        }
    }
    
    //check if joint velocitys are in range and if necessary adjust them by adding points.
    CheckVelocities(trajectory, _positions);
    
    //crop trajectory because first Configuration* is current configuration of robot.
    trajectory->startAt(1);
    
    //add impossible configuration as help for detection of aborted path.
    if(pathCancelled)
        trajectory->add_configuration(new Configuration({0, 0, 0, 0, 0, 0}));

    return trajectory;
}

Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig, bool weight)
{
    double minDist = 1000;
    int minConfig = 0;
    
    for (int i = 0; i < _configs->size(); i++) {
        Configuration* ActConfig = _configs->at(i);
        double actDist = 0;
        for (int j = 0; j < NUM_JOINTS; j++) {
            if(weight && j < 3){
                actDist += 2*pow((*ActConfig)[j] - (*_prevConfig)[j], 2);
            }
            else actDist += pow((*ActConfig)[j] - (*_prevConfig)[j], 2);
        }
        actDist = sqrt(actDist);

        if (actDist < minDist){
            minDist = actDist;
            minConfig = i;
        }
    }
    return _configs->at(minConfig);
}

Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _endConfig, Configuration* _wristConfig)
{
    //merge _prevConfig(arm) and _wrist Config(wrist)
    std::array<double, NUM_JOINTS> merged;
    for(int i= 0; i < 3; i++)
    {
        merged[i]= (*_endConfig)[i];
        merged[i+3]=(*_wristConfig)[i+3];
    }
    Configuration* newConfig = new Configuration(merged);
    Configuration* closest = GetClosestConfiguration(_configs, newConfig, true);

    if(std::abs((*_wristConfig)[0]-(*closest)[0]) < 5){
        return closest;
    }
    else return _wristConfig;
}

void IVMovement::CheckVelocities(Trajectory* _trajectory, vector<SixDPos*>* _positions)
{
    int NbEx = 0; //Number of exceptions.
    bool exception;
    
    //Check joint velocities that are too high due to distance between points and
    //add points to reduce the velocity. Differences above a certain threshold are
    //ignored because those are flip points (change from one case to another resulting
    //in high rotations in joints that can not be reduced by adding points)
    for(int i = 1; i < _trajectory->get_length()+NbEx; i++)
    {
        exception = false;
        for(int j = 0; j < 6; j++)
        {
            Configuration* current = _trajectory->get_configuration(i-NbEx);
            Configuration* previous = _trajectory->get_configuration(i-1-NbEx);
            
            if(std::abs(((*current)[j] - (*previous)[j]) /robot->time_interval)
               > robot->velocities[j]){
                if (getMaxDiff(current,previous) < 0.5*M_PI){
                    exception = true;
                }
            }
        }
        if(exception){
            if(Interpolate(_trajectory, _positions, i-NbEx)) NbEx++;
            if(NbEx > 1000) break; //prevention of endless loop.
        }
    }

    if(NbEx > 0){
        cout << "Too high velocities detected, corresponding points changed." << endl;
    }
    
    //Detect flip points of joints (change from one case to another) and interpolate
    //configuration at these points to reduce the velocity. Interpolated values are
    //displayed red on the path in the simulation and therefore need to be stored.
    NbEx = 0;
    std::vector<SixDPos*> temp;
    std::vector<int> LoopSize;
    std::vector<int> LoopStart;
    int actSize = 0;
    
    for(int i = 1; i < _trajectory->get_length()+NbEx; i++)
    {
        exception = false;
        for(int j = 0; j < 6; j++)
        {
            if(std::abs(((*_trajectory->get_configuration(i-NbEx))[j] -
                 ((*_trajectory->get_configuration(i-1-NbEx))[j]))
                /robot->time_interval) > robot->velocities[j]){
                exception = true;
            }
        }
        if(exception){
            //store start index and size of each flipping loop.
            if(JointInterpolate(trajectory, i-NbEx)){
                if(LoopStart.size()==0){
                    LoopStart.push_back(i-NbEx);
                }
                else if(i-NbEx - (LoopStart.back()+actSize) > 3){
                    LoopSize.push_back(actSize+1);
                    actSize = 0;
                    LoopStart.push_back(i-NbEx);
                }
                else actSize++;
                NbEx++;
                if(NbEx > 1000) break;
            }
        }
    }
    
    LoopSize.push_back(actSize+1);
    
    if(loopVector) getLoopPoints(LoopStart, LoopSize);
    
    if(NbEx > 0){
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
    
    vector<Configuration*>* configs = invK->get_inv_kinematics(PosC);
    
    if(configs->size()>0)
    {
        Configuration* correctConfig = GetClosestConfiguration(configs, trajectory->get_configuration(index-1));
        
        //check for overhead singularity.
        if(overheadSingularity(PosC)){
            correctConfig = osInterpolation(trajectory->get_configuration(index-1),
                                  correctConfig,
                                  trajectory->get_configuration(index), PosA, PosB, PosC);
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

void IVMovement::wsInterpolation(Trajectory* trajectory, int width)
{
    int size = trajectory->get_length();
    
    Configuration* startConfig = trajectory->get_configuration(size-width-1);
    Configuration* endConfig = trajectory->get_configuration(size-1);
    
    std::array<double, NUM_JOINTS> newConfig {};

    for(int i = 0; i < width; i++){
        Configuration* actConfig = trajectory->get_configuration(size-width+i);
        for(int j= 0; j < 3; j++){
            newConfig[j] = (*actConfig)[j];
            newConfig[j+3] = (*startConfig)[j+3] + (i+1)/(double)(width)*((*endConfig)[j+3]-(*startConfig)[j+3]);
        }
        
        trajectory->set_configuration(new Configuration(newConfig), size-width+i);
    }
}

Configuration* IVMovement::osInterpolation(Configuration *startConfig, Configuration *curConfig, Configuration *endConfig, SixDPos* posA, SixDPos* posB, SixDPos* posC)
{
    Trajectory* A = new Trajectory();
    vector<SixDPos*>* positions = new vector<SixDPos*>();
    
    A->add_configuration(startConfig);
    A->add_configuration(curConfig);
    A->add_configuration(endConfig);
    
    positions->push_back(posA);
    positions->push_back(posB);
    positions->push_back(posC);
        
    osInterpolation(A, 2, positions);
    
    return A->get_configuration(1);
}

void IVMovement::osInterpolation(Trajectory* trajectory, int width, vector<SixDPos*>* _positions)
{
    int size = trajectory->get_length();
    std::array<double, 3> newConfig {};
    Configuration* startConfig = trajectory->get_configuration(size-width-1);
    Configuration* endConfig = trajectory->get_configuration(size-1);
    
    //if there is a flip (eg. 180 degree rotation) at the singularity
    //the maximal and minimal values have to be represented in the singularity
    //region for correct interpolation.
    if(std::abs((*startConfig)[0]-(*endConfig)[0]) > 0.5*M_PI && width > 2){
        for(int i = 0; i < width-1; i++){
            Configuration* actConfig = trajectory->get_configuration(size-width+i);
            newConfig[0] = (*startConfig)[0] + i/(double)(width-2)*((*endConfig)[0]-(*startConfig)[0]);
            newConfig[1] = (*actConfig)[1];
            newConfig[2] = (*actConfig)[2];

            vector<Configuration*>* configs = invK->get_inv_kinematics(_positions->at(size-width+i),
                                                                       &newConfig);
            Configuration* correctConfig =  GetClosestConfiguration(configs, trajectory->get_configuration(size-width+i-1));

            trajectory->set_configuration(correctConfig, size-width+i);
        }
    }
    else{
        for(int i = 0; i < width-1; i++){
            Configuration* actConfig = trajectory->get_configuration(size-width+i);
            newConfig[0] = (*startConfig)[0] + (i+1)/(double)(width)*((*endConfig)[0]-(*startConfig)[0]);
            newConfig[1] = (*actConfig)[1];
            newConfig[2] = (*actConfig)[2];

            vector<Configuration*>* configs = invK->get_inv_kinematics(_positions->at(size-width+i),
                                                                       &newConfig);
            Configuration* correctConfig =  GetClosestConfiguration(configs, trajectory->get_configuration(size-width+i-1));

            trajectory->set_configuration(correctConfig, size-width+i);
        }
    }
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
    //The singulartiy at joint5 == pi has to be considered
    //because of the boundary conditions. 
    if(std::abs((*_config)[4])< SINGULARITY_MARGIN) return true;
    else if(std::abs(std::abs((*_config)[4]) -M_PI)< SINGULARITY_MARGIN) return true;
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

void IVMovement::wsLastConfig(Configuration* Config, int width)
{
    Configuration* lastConfig = trajectory->get_configuration(trajectory->get_length()-1-width);
    double A = (*Config)[3] + (*Config)[5];
    double B = (*lastConfig)[3] + (*lastConfig)[5];
    double diff = A - B;
    
    (*Config)[3] = (*lastConfig)[3] + 0.5* diff;
    (*Config)[5] = (*lastConfig)[5] + 0.5* diff;
    
    trajectory->set_configuration(Config, trajectory->get_length()-1);
}

void IVMovement::osLastConfig(Configuration* Config, SixDPos* Pos, int width)
{
    Configuration* lastConfig = trajectory->get_configuration(trajectory->get_length()-1-width);
    (*Config)[0] = (*lastConfig)[0];
    
    //calculate wrist configuration for new arm configuration.
    vector<Configuration*>* configs = invK->get_inv_kinematics(Pos, Config);
    
    if(configs->size()>0){
        Configuration* correctConfig =  GetClosestConfiguration(configs, lastConfig);
        trajectory->set_configuration(correctConfig, trajectory->get_length()-1);
    }
    else trajectory->set_configuration(Config, trajectory->get_length()-1);
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
