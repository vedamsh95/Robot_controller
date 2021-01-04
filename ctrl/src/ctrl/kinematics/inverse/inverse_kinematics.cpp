#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>

std::array<double, 4> wcp;

double o = 115;
double m = 330;
double n = 645;
double a = 1150;
double b = 1220;

std::array<double, 6> solution;
std::array<double, 5> phi2_phi3;
std::array<double, 6> sol;

std::array<double, 6> InvKinematics::inv_standardcase(double phi1, double d1) {
    std::array<double, 6> solution;

    if (-185 < phi1 < -175) {
        cout << "needs to be implemented" << endl;
    }
    else if (185 > phi1 && phi1 > 175) {
        cout << "needs to be implemented" << endl;
    }
    else if (-5 < phi1 && 5 > phi1) {
        cout << "needs to be implemented" << endl;
    }
    else if (d1 > m && -175 < phi1 < 175)                                                                               //calculating all the forward cases
    {
        std::cout << "forward case " << std::endl;
        double dpx = d1 - m;                                                                                            //calculating the x and y components of the
        cout << "dpx: " << dpx << endl;
        double dpy = wcp[2] - n;
        cout << "dpy: " << dpy << endl;

        inv_forwardcase(dpx, dpy);
        sol = inv_checklimits(phi1);
        return sol;
    }                                                                                        //distance between the second joint and wcp

    else if (d1 < m && -175 < phi1 < 175)                              //calculating all the backward cases
    {
        std::cout << "backward case " << std::endl;
        double dpx = m - d1;                                  //calculating the x and y components of the
        std::cout << "dpx: " << dpx << std::endl;
        double dpy = wcp[2] - n;                              //distnance between the second joint and wcp
        std::cout << "dpy: " << dpy << std::endl;
        inv_backwardcase(dpx, dpy);
        sol = inv_checklimits(phi1);
        return sol;
    }
}

std::array<double, 5> InvKinematics::inv_forwardcase(double dpx, double dpy){

    double d3 = sqrt(dpx * dpx + dpy * dpy);                                                                        //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2))*180/M_PI);
    std::cout << "beta: " <<  beta << std::endl;
    double alpha1 = (asin(sin(beta) * (d2 / d3)))*180/M_PI;
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3))*180/M_PI;
    std::cout << "alpha2: " << alpha2 << std::endl;

    double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
    cout << "phi2_f_u: " << phi2_f_u << endl;
    double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down
    cout << "phi2_f_d: " << phi2_f_d << endl;

    double phi3_f_u = 360 - beta - asin(b / d2)*180/M_PI - 90;                                                            //for forward elbow up
    cout << "phi3_f_u: " << phi3_f_u << endl;
    double phi3_f_d = -1*(90-(360-(asin(b/d2))*180/M_PI-90));                                                                  //for forward elbow down
    cout << "phi3_f_d: " << phi3_f_d << endl;
    double phi3_f_d_1= -1*(90+asin(b/d2)*180/M_PI-beta);
    cout << "phi3_f_d1: " << phi3_f_d_1 << endl;

    phi2_phi3 = {phi2_f_u, phi2_f_d, phi3_f_u, phi3_f_d, phi3_f_d_1};

    return phi2_phi3;
}

std::array<double, 5> InvKinematics::inv_backwardcase(double dpx, double dpy) {
    double d3 = sqrt(dpx * dpx + dpy * dpy);                   //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2))*180/M_PI);
    std::cout << "beta: " <<  beta << std::endl;
    double alpha1 = (asin(sin(beta) * (d2 / d3)))*180/M_PI;
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3))*180/M_PI;
    std::cout << "alpha2: " << alpha2 << std::endl;

    double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
    std::cout << "phi2_b_u: " << phi2_b_u << std::endl;
    double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down
    std::cout << "phi2_b_d: " << phi2_b_d << std::endl;

    double phi3_b_u = 270 - beta - (asin(b / d2))*180/M_PI;              //for backwards elbow up
    std::cout << "phi3_b_u: " << phi3_b_u << std::endl;
    double phi3_b_d = (360-beta-asin(b/d2)*180/M_PI-90);                                                             //double phi3_b_d = -1 * (90 - (beta - asin(b / d2)*180/M_PI));        //for backwards elbow down
    std::cout << "phi3_b_d: " << phi3_b_d << std::endl;
    double phi3_b_d_1 = 270-beta-asin(b/d2)*180/M_PI;
    std::cout << "phi3_b_d_1: " << phi3_b_d_1 << std::endl;

    phi2_phi3 = {phi2_b_u, phi2_b_d, phi3_b_u, phi3_b_d, phi3_b_d_1};

    return phi2_phi3;
}

std::array<double, 6> InvKinematics::inv_checklimits(double phi1){

    if (-185 < phi1 && phi1 < 185) {
        std::cout << "phi1: " << phi1 << std::endl;
        if (-140 < phi2_phi3[0] && phi2_phi3[0] < -5) {
            std::cout << "phi2: " << phi2_phi3[0] << std::endl;
            if (-120 < phi2_phi3[2] && phi2_phi3[2] < 168) {
                std::cout << "phi3: " << phi2_phi3[2] << std::endl;
                solution[0] = phi1;
                solution[1] = phi2_phi3[0];
                solution[2] = phi2_phi3[2];
                return solution;
            }
        }
        else if (-140 < phi2_phi3[1] < -5) {
            std::cout << "phi2: " << phi2_phi3[1] << std::endl;
            if (-120 < phi2_phi3[3] && phi2_phi3[3] < 168) {
                std::cout << "phi3: " << phi2_phi3[3] << std::endl;
                solution[0] = phi1;
                solution[1] = phi2_phi3[1];
                solution[2] = phi2_phi3[3];
                return solution;
            }
            else if(-120 < phi2_phi3[4] && phi2_phi3[4] < 168){
                std::cout << "phi3: " << phi2_phi3[4] << std::endl;
                solution[0] = phi1;
                solution[1] = phi2_phi3[1];
                solution[2] = phi2_phi3[4];
                return solution;
            }
        }
    }
}

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!

    double phi1;

    std::array<double, 6> solution;
    std::array<double, 4> dTCP = {0,0,-215,1};                                                                        //Vector of the distance from TCP to WCP


    std::cout << "X of sixDPos" <<_pos->get_X() << std::endl;
    std::cout << "Y of sixDPos" <<_pos->get_Y() << std::endl;
    std::cout << "Z of sixDPos" <<_pos->get_Z() << std::endl;
    std::cout << "A of sixDPos" <<_pos->get_A() << std::endl;
    std::cout << "B of sixDPos" <<_pos->get_B() << std::endl;
    std::cout << "C of sixDPos" <<_pos->get_C() << std::endl;


    TMatrix TCP(_pos->get_A(),_pos->get_B(),_pos->get_C(),_pos->get_X()*1000,_pos->get_Y()*1000,_pos->get_Z()*1000);                                                            //Transformation Matrix for the TCP inside of the global coordinate system
    //std::array<double, 4> wcp = TCP*dTCP;                                                                                                                                                 //Calculation of wrist center point

    wcp[0]= (_pos->get_X()*1000)-215*(sin(_pos->get_C()) * sin(_pos->get_A()) + cos(_pos->get_C())*sin(_pos->get_B())*cos(_pos->get_A()));
    wcp[1]= (_pos->get_Y()*1000)-215*((-1)*cos(_pos->get_C()) * sin(_pos->get_A()) + sin(_pos->get_C())*sin(_pos->get_B()) * sin(_pos->get_A()));
    wcp[2]= (_pos->get_Z()*1000)-215*(cos(_pos->get_B()) * cos(_pos->get_A()));


    for (int i = 0; i < 4; ++i) {
        std::cout << "wcp :" << wcp[i] << std::endl;
    }


    double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
    std::cout << "d1: " << d1 << std::endl;

    if(wcp[0]>0 && wcp[1]>=0)
    {
        cout << "Fall 1" << endl;
        phi1= (-atan(wcp[1]/wcp[0]))*180/M_PI;
        cout << "phi1: " << phi1 << endl;
        solution = inv_standardcase(phi1, d1);

    }

    else if(wcp[0]<0 && wcp[1]<=0)
    {
        cout << "Fall 2" << endl;
        phi1= 180-atan(wcp[1]/wcp[0])*180/M_PI;
        cout << "phi1: " << phi1 << endl;
        solution = inv_standardcase(phi1, d1);

    }

    else if(wcp[0]>0 && wcp[1]<=0)
    {
        cout << "Fall 3" << endl;
        phi1=atan(wcp[1]/wcp[0])*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        solution = inv_standardcase(phi1, d1);

    }

    else if(wcp[0]<0 && wcp[1]>=0)
    {
        cout << "Fall 4" << endl;
        phi1=180-atan(wcp[1]/-wcp[0])*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        solution = inv_standardcase(phi1, d1);
    }

        //Special cases
        //Special case 1:
    else if(wcp[0]==0 && wcp[1]>0)
    {
        cout << "needs to be implemented" << endl;
    }
        //special case 2:
    else if(wcp[0]==0 && wcp[1]<0)
    {
        cout << "needs to be implemented" << endl;
    }



    std::cout<< "Solution_phi1:" << solution[0] << endl;
    std::cout<< "Solution_phi2:" << solution[1] << endl;
    std::cout<< "Solution_phi3:" << solution[2] << endl;

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