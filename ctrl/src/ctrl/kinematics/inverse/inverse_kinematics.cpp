#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"


InvKinematics::InvKinematics(){
    robot = &Robot::getInstance();
}

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos, bool setJointLimits)
{
   //TODO: IMPLEMENT Compute the inverse kinematics for a given position
    vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->clear();
    
    //TODO: Get first three joints from IVKinPos
	IVKinPos Position;
    vector<array<double, 3>*>* IVpos = new vector<array<double, 3>*>();
    //IVpos->clear();
	IVpos = Position.get_IVKinPos(_pos);
    
    //calcualte rotation matrix for all joints.
    TMatrix R_06 = TMatrix(_pos->get_C(),
                           _pos->get_B(),
                           _pos->get_A(),
                           _pos->get_X(),
                           _pos->get_Y(),
                           _pos->get_Z());


    for (int i = 0; i < IVpos->size(); i++)
    {

        array<double, 3>* actPos = new array<double, 3>;
        actPos = IVpos->at(i);
        
        //convert actPos from degree to radian
        for (int j =0; j <3; j++)
        {
            actPos->at(j) = actPos->at(j)*M_PI/180;
        }
        
        //calculate rotation matrix for first 3 joints using fwKinematics.
        FwKinematics fw;

        // These are the first four Denavit-Hartenberg parameters for our robot
        double dh_table[4][4] = {
                {0,       M_PI,   0,    645},
                {0,       M_PI_2, 330,  0},
                {0, 0,            1150, 0},
                {-M_PI_2, M_PI_2, 115,  0},
        };

        // Transformation matrix from the base coordinate system
        // to the coordinate system of the first joint. This does
        // not depend on any variables.
        TMatrix R_03 = fw.create_single_t_matrix(
                dh_table[0][0],
                dh_table[0][1],
                dh_table[0][2],
                dh_table[0][3]
        );

        // Create and multiply the individual transformation matrices
        // from one joint to the next one.
        for (int j = 0; j < 3; j++) {
            TMatrix next = fw.create_single_t_matrix(
                    dh_table[j + 1][0] + actPos->at(j),
                    dh_table[j + 1][1],
                    dh_table[j + 1][2],
                    dh_table[j + 1][3]
            );
            R_03 = R_03.multiply(next);
        }

        //calculate rotation matrix for last 3 joints.
        TMatrix R_36 = (R_03.transpose()).multiply(R_06);

        double theta4[4], theta5[4], theta6[4];
        
        theta4[0] = atan2(-R_36.get(1, 2), -R_36.get(0, 2));
        theta5[0] = atan2(sqrt(1-pow(R_36.get(2, 2), 2)), -R_36.get(2, 2));
        theta6[0] = atan2(R_36.get(2, 1), R_36.get(2, 0));
        
        theta4[1] = theta4[0] - sign(theta4[0])*2*M_PI;
        theta6[1] = theta6[0] - sign(theta6[0])*2*M_PI;
        
        theta4[2] = atan2(R_36.get(1, 2), R_36.get(0, 2));
        theta5[2] = atan2(-sqrt(1-pow(R_36.get(2, 2), 2)), -R_36.get(2, 2));
        theta6[2] = atan2(-R_36.get(2, 1), -R_36.get(2, 0));
        
        theta4[3] = theta4[2] - sign(theta4[2])*2*M_PI;
        theta6[3] = theta6[2] - sign(theta6[2])*2*M_PI;
        
        double Configs[8][3]=
        {
            {theta4[0], theta5[0], theta6[0]},
            {theta4[0], theta5[0], theta6[1]},
            {theta4[1], theta5[0], theta6[0]},
            {theta4[1], theta5[0], theta6[1]},
            {theta4[2], theta5[2], theta6[2]},
            {theta4[2], theta5[2], theta6[3]},
            {theta4[3], theta5[2], theta6[2]},
            {theta4[3], theta5[2], theta6[3]},
        };
        
        if(!setJointLimits)
        {
            for (int j = 0; j < 8;j++)
            {
                if ((j<4 && Configs[j][1] > singularityMargin && Configs[j][1] < JOINT_5_MAX) ||
                    (j>3 && Configs[j][1] < -singularityMargin && Configs[j][1] > -JOINT_5_MAX)){
                    if(std::abs(Configs[j][0]) < robot->limits[3].max
                                       && std::abs(Configs[j][2]) < robot->limits[5].max){
                                        solutions->push_back(new Configuration({
                                            actPos->at(0),
                                            actPos->at(1),
                                            actPos->at(2),
                                            Configs[j][0],
                                            Configs[j][1],
                                            Configs[j][2]}));
                                    }
                }
                else if((j<4 && Configs[j][1] < singularityMargin && Configs[j][1] > 0) ||
                        (j>4 && Configs[j][1] > -singularityMargin && Configs[j][1] < 0))
                {
                    //cout << "Wrist Singularity Detected!" << endl;
                    array<double, 3>* wristRotation = new array<double, 3>;
                    wristRotation = CalculateSingularity(R_36);
                    
                    if(wristRotation->at(0) < robot->limits[3].max
                       && wristRotation->at(2) < robot->limits[5].max){
                        solutions->push_back(new Configuration({
                            actPos->at(0),
                            actPos->at(1),
                            actPos->at(2),
                            Configs[j][0],
                            wristRotation->at(1),
                            Configs[j][2]}));
                    }
                }
            }
        }
        else
        {
            for(int j=0; j<8; j++){
                setToLimits(&Configs[j][1], j);
                if(std::abs(Configs[j][0]) < robot->limits[3].max
                                   && std::abs(Configs[j][2]) < robot->limits[5].max){
                                    solutions->push_back(new Configuration({
                                        actPos->at(0),
                                        actPos->at(1),
                                        actPos->at(2),
                                        Configs[j][0],
                                        Configs[j][1],
                                        Configs[j][2]}));
                                }
            }
        }
    }
    
    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    
    //vector<Configuration*>* solutions = new vector<Configuration*>();
//    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));


    return solutions;
}

array<double, 3>* InvKinematics::CalculateSingularity(TMatrix R36)
{
    array<double, 3>* wristRotation = new array<double, 3>;
    wristRotation->at(0)= 0.5*acos(R36.get(1, 1));
    wristRotation->at(1)= 0;
    wristRotation->at(2)= 0.5*acos(R36.get(1, 1));
    
    return wristRotation;
}

void InvKinematics::setToLimits(double* theta5, int j)
{
    if(j<4){
        if(*theta5 < singularityMargin)
            *theta5=0;
        else if(*theta5 > JOINT_5_MAX)
            *theta5 = JOINT_5_MAX;
    }
    else
        if(*theta5 > singularityMargin)
            *theta5=0;
        else if(*theta5 < -JOINT_5_MAX)
            *theta5 = -JOINT_5_MAX;
}
    

double InvKinematics::sign(double value)
{
    if(value > 0) return 1;
    if(value < 0) return -1;
    return 0;
}
