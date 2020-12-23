#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
#include <vector>


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
//  //TODO: IMPLEMENT Compute the inverse kinematics for a given position
    vector<Configuration*>* solutions = new vector<Configuration*>();
    
    //TODO: Get first three joints from IVKinPos
//    IVKinPos Position(_pos->get_X(), _pos->get_Y(), _pos->get_Z());
    vector<array<double, 3>*>* IVpos = new vector<array<double, 3>*>();
//    IVpos = Position.get_IVKinPos();
    
    
    //Test values:
    array<double, 3> a1, a2;
    a1 = {0, 0, 0};
    a2 = {1, 2, 3};
    IVpos->push_back(&a1);
    IVpos->push_back(&a2);
    

    for (int i = 0; i < IVpos->size(); i++)
    {

        array<double, 3>* actPos = new array<double, 3>;
        actPos = IVpos->at(i);

        //calculate rotation matrix for first 3 joints.
        TMatrix R_03(actPos->at(0), actPos->at(1), actPos->at(2), 0, 0, 0);
        cout << "Matrix R_03: " << R_03 << endl;

        //calcualte rotation matrix for all joints.
        TMatrix R_06 = TMatrix(_pos->get_A(), _pos->get_B(), _pos->get_C(), _pos->get_X(), _pos->get_Y(), _pos->get_Z());
        cout << "Matrix R_06: " << R_06 << endl;

        //calculate rotation matrix for last 3 joints.
        TMatrix R_36 = (R_03.transpose()).multiply(R_06);

        double theta4[4], theta5[4], theta6[4];
        for (int i = 0; i < 2; i++)
        {
            theta4[i] = atan2(-R_36.get(1, 2), -R_36.get(0, 2)) + 2* M_PI * i;
            theta5[i] = atan2(sqrt(1-pow(R_36.get(2, 2), 2)), -R_36.get(2, 2));
            theta6[i] = atan2(R_36.get(2, 1), R_36.get(2, 0)) + 2* M_PI * i;

            theta4[i+2] = atan2(R_36.get(1, 2), R_36.get(0, 2)) + 2* M_PI * i;
            theta5[i+2] = atan2(-sqrt(1-pow(R_36.get(2, 2), 2)), -R_36.get(2, 2));
            theta6[i+2] = atan2(-R_36.get(2, 1), -R_36.get(2, 0)) + 2* M_PI * i;
        }

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


        for (int i = 0; i < 8; i++)
        {
            solutions->push_back(new Configuration({actPos->at(0), actPos->at(1),actPos->at(2),Configs[i][0],Configs[i][1],Configs[i][2]}));
        }

    }
    
    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    
    //vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));

    return solutions;
}



