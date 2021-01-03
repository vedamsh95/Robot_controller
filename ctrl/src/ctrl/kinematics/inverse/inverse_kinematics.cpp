#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!

    std::cout << "X of sixDPos" <<_pos->get_X() << std::endl;
    std::cout << "Y of sixDPos" <<_pos->get_Y() << std::endl;
    std::cout << "Z of sixDPos" <<_pos->get_Z() << std::endl;
    std::cout << "A of sixDPos" <<_pos->get_A() << std::endl;
    std::cout << "B of sixDPos" <<_pos->get_B() << std::endl;
    std::cout << "C of sixDPos" <<_pos->get_C() << std::endl;


    vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->push_back(new Configuration({0,0,1,0,0,0}));
    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));

    return solutions;
}