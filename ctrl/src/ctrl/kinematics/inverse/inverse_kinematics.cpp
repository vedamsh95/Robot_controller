#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <cmath>
#include <iostream>

double o = 115;
double m = 330;
double n = 645;
double a = 1150;
double b = 1220;

std::array<double, 6> solution;
std::array<double, 4> phi2_phi3;
std::array<double, 6> solution_standard;
std::array<double, 6> sol_standard;

std::array<double, 6> sol_specialcase1_1;
std::array<double, 6> sol_specialcase1_2;
std::array<double, 6> sol_specialcase2_1;
std::array<double, 6> sol_specialcase2_2;

std::array<double, 6> sol_phi1special1_1;
std::array<double, 6> sol_phi1special1_2;
std::array<double, 6> sol_phi1special1_3;

double _Trans03[6];

std::array<double, 10> InvKinematics::inv_gettheta4_5_6(TMatrix R36){

    //components of R36 needed for the calculations of Theta 4, 5, 6:
    double ax = R36.get_element(0,2);
    double ay = R36.get_element(1,2);
    double az = R36.get_element(2,3);
    double sz = R36.get_element(2,1);
    double nz = R36.get_element(2,0);

    //Calculations for the varieties of thetas
    array<double, 4> theta4;
    array<double, 2> theta5;
    array<double, 4> theta6;

    //For theta 5 > 0
    theta4[0] = atan2(-ay,-ax)*180/M_PI;
    theta4[1] = 360 + theta4[0];
    theta5[0] = atan2(sqrt(1-pow((az),2)), -az)*180/M_PI;
    theta6[0] = atan2(sz,nz)*180/M_PI;
    theta6[1] = 360 + theta6[0];

    //For theta 5 < 0
    theta4[2] = atan2(ay,ax)*180/M_PI;
    theta4[3] = -360 + theta4[2];
    theta5[1] = atan2(-1*sqrt(1-pow((az),2)), -az)*180/M_PI;
    theta6[2] = atan2(-sz,-nz)*180/M_PI;
    theta6[3] = -360 + theta6[2];

    std::array<double,10> theta4_5_6 = {theta4[0],theta4[1],theta4[2],theta4[3],
                                        theta5[0],theta5[1],
                                        theta6[0],theta6[1],theta6[2],theta6[3]};

    return theta4_5_6;
}

std::array<double, 9> InvKinematics::inv_checktheta(double phi1, double d1, std::array<double, 3> wcp) {
    if(-185 < phi1 < -175) {
        //cout << "needs to be implemented" << endl;
        double dpx = d1 - m;
        double dpy = wcp[2] - n;

        if(d1 > m && wcp[2] > n){
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 + 360;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_2 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 + 180;
            dpx = d1 + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits(phi1, phi2_phi3);

            std::array<double, 9> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
                                                   sol_phi1special1_1[2], sol_phi1special1_2[0],
                                                   sol_phi1special1_2[1],sol_phi1special1_2[3],
                                                   sol_phi1special1_3[0],sol_phi1special1_3[1],
                                                   sol_phi1special1_3[2]};
            return sol_phi1specialcases;
            }

        if(d1 < m){
            dpx = m - d1;
            dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 + 360;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 + 180;
            dpx = d1 + m;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits(phi1, phi2_phi3);

            std::array<double, 9> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
                                                       sol_phi1special1_1[2], sol_phi1special1_2[0],
                                                       sol_phi1special1_2[1],sol_phi1special1_2[3],
                                                       sol_phi1special1_3[0],sol_phi1special1_3[1],
                                                       sol_phi1special1_3[2]};
            return sol_phi1specialcases;
        }
    }

    else if (185 > phi1 && phi1 > 175) {
        //cout << "needs to be implemented" << endl;
        double dpx = d1 - m;
        double dpy = wcp[2] - n;

        if(d1 > m && wcp[2] > n){
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 - 360;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_2 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 - 180;
            dpx = d1 + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits(phi1, phi2_phi3);

            std::array<double, 9> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
                                                       sol_phi1special1_1[2], sol_phi1special1_2[0],
                                                       sol_phi1special1_2[1],sol_phi1special1_2[3],
                                                       sol_phi1special1_3[0],sol_phi1special1_3[1],
                                                       sol_phi1special1_3[2]};
            return sol_phi1specialcases;

        }

        if(d1 < m){
            dpx = m - d1;
            dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);

            phi1 = phi1 - 360;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);

            phi1 = phi1 - 180;
            dpx = d1 + m;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits(phi1, phi2_phi3);

            std::array<double, 9> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
                                                       sol_phi1special1_1[2], sol_phi1special1_2[0],
                                                       sol_phi1special1_2[1],sol_phi1special1_2[3],
                                                       sol_phi1special1_3[0],sol_phi1special1_3[1],
                                                       sol_phi1special1_3[2]};
            return sol_phi1specialcases;

        }
    }
    else if (-5 < phi1 && 5 > phi1) {
        cout << "needs to be implemented" << endl;
    }
}

std::array<double, 6> InvKinematics::inv_standardcase(double phi1, double d1, std::array<double, 3> wcp) {
    std::array<double, 6> solution;
   //checking for special phi1 configurations
   //standard cases
    if (d1 > m && -175 < phi1 < 175)                                                                               //calculating all the forward cases
    {
        std::cout << "forward case " << std::endl;
        double dpx = d1 - m;                                                                                            //calculating the x and y components of the
        cout << "dpx: " << dpx << endl;
        double dpy = wcp[2] - n;
        cout << "dpy: " << dpy << endl;

        phi2_phi3 = inv_forwardcase(dpx, dpy);
        sol_standard = inv_checklimits(phi1, phi2_phi3);
        return sol_standard;

    }                                                                                        //distance between the second joint and wcp

    else if (d1 < m && -175 < phi1 < 175)                              //calculating all the backward cases
    {
        std::cout << "backward case " << std::endl;
        double dpx = m - d1;                                  //calculating the x and y components of the
        std::cout << "dpx: " << dpx << std::endl;
        double dpy = wcp[2] - n;                              //distnance between the second joint and wcp
        std::cout << "dpy: " << dpy << std::endl;
        phi2_phi3 = inv_backwardcase(dpx, dpy);
        sol_standard = inv_checklimits(phi1, phi2_phi3);
        return sol_standard;
    }
}

std::array<double, 4> InvKinematics::inv_forwardcase(double dpx, double dpy){

    double d3 = sqrt(dpx * dpx + dpy * dpy);                                                                        //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)));
    std::cout << "beta: " <<  beta << std::endl;
    double pre_alpha1 = sin(beta)*(d2/d3);
    std::cout << "pre_alpha1: " << pre_alpha1 << endl;
    double alpha1 = (asin(pre_alpha1)*180/M_PI);
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3))*180/M_PI;
    std::cout << "alpha2: " << alpha2 << std::endl;

    double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
    cout << "phi2_f_u: " << phi2_f_u << endl;
    double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down
    cout << "phi2_f_d: " << phi2_f_d << endl;

    double phi3_f_u = 360 - beta*180/M_PI - asin(b / d2)*180/M_PI - 90;                                                            //for forward elbow up
    cout << "phi3_f_u: " << phi3_f_u << endl;
    double phi3_f_d = beta*180/M_PI - ((asin(b/d2))*180/M_PI)-90;                                                                  //for forward elbow down
    cout << "phi3_f_d: " << phi3_f_d << endl;

    phi2_phi3 = {phi2_f_u, phi2_f_d, phi3_f_u, phi3_f_d};

    return phi2_phi3;
}

std::array<double, 4> InvKinematics::inv_backwardcase(double dpx, double dpy) {
    double d3 = sqrt(dpx * dpx + dpy * dpy);                   //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)));
    std::cout << "beta: " <<  beta << std::endl;
    double alpha1 = (asin(sin(beta) * (d2 / d3)));
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3));
    std::cout << "alpha2: " << alpha2 << std::endl;

    double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
    std::cout << "phi2_b_u: " << phi2_b_u << std::endl;
    double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down
    std::cout << "phi2_b_d: " << phi2_b_d << std::endl;

    double phi3_b_u = 270 - beta*180/M_PI-(asin(b / d2)*180/M_PI);              //for backwards elbow up
    std::cout << "phi3_b_u: " << phi3_b_u << std::endl;
    double phi3_b_d = -1*(90-(beta-asin(b/d2)*180/M_PI));                                                             //double phi3_b_d = -1 * (90 - (beta - asin(b / d2)*180/M_PI));        //for backwards elbow down
    std::cout << "phi3_b_d: " << phi3_b_d << std::endl;

    phi2_phi3 = {phi2_b_u, phi2_b_d, phi3_b_u, phi3_b_d};

    return phi2_phi3;
}

std::array<double, 6> InvKinematics::inv_checklimits(double phi1, array<double, 4> phi2_phi3){
bool elbowup = false;
bool elbowdown = false;
    if (-185 < phi1 && phi1 < 185) {
        std::cout << "phi1: " << phi1 << std::endl;
        if (-140 < phi2_phi3[0] && phi2_phi3[0] < -5) {
            std::cout << "phi2: " << phi2_phi3[0] << std::endl;
            if (-120 < phi2_phi3[2] && phi2_phi3[2] < 168) {
                std::cout << "phi3: " << phi2_phi3[2] << std::endl;
                elbowup = true;
//                solution[0] = phi1;
//                solution[1] = phi2_phi3[0];
//                solution[2] = phi2_phi3[2];
               // return solution;

            }

        }
        if (-140 < phi2_phi3[1] && phi2_phi3[1]< -5) {
            std::cout << "phi2: " << phi2_phi3[1] << std::endl;
            if (-120 < phi2_phi3[3] && phi2_phi3[3] < 168) {
                std::cout << "phi3: " << phi2_phi3[3] << std::endl;
//                solution[3] = phi1;
//                solution[4] = phi2_phi3[1];
//                solution[5] = phi2_phi3[3];
//                return solution;
        elbowdown = true;
            }
        }
    }
    if (elbowup == true && elbowdown == true){
        solution[0] = phi1;
        solution[1] = phi2_phi3[0];
        solution[2] = phi2_phi3[2];
        solution[3] = phi1;
        solution[4] = phi2_phi3[1];
        solution[5] = phi2_phi3[3];
        std::cout << "Both Configurations are true" << endl;
        return solution;
    }

    else if(elbowup == true && elbowdown == false){
        solution[0] = phi1;
        solution[1] = phi2_phi3[0];
        solution[2] = phi2_phi3[2];
        solution[3] = 0;
        solution[4] = 0;
        solution[5] = 0;
        std::cout << "";
    }
}

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    //you should call your inverse kinematics functions here!

    double phi1;
    std::array<double, 4> dTCP = {0,0,-215,1};                                                                        //Vector of the distance from TCP to WCP

    std::cout << "X of sixDPos" <<_pos->get_X() << std::endl;
    std::cout << "Y of sixDPos" <<_pos->get_Y() << std::endl;
    std::cout << "Z of sixDPos" <<_pos->get_Z() << std::endl;
    std::cout << "A of sixDPos" <<_pos->get_A() << std::endl;
    std::cout << "B of sixDPos" <<_pos->get_B() << std::endl;
    std::cout << "C of sixDPos" <<_pos->get_C() << std::endl;

    TMatrix TCP(_pos->get_A(),_pos->get_B(),_pos->get_C(),_pos->get_X()*1000,_pos->get_Y()*1000,_pos->get_Z()*1000);                                                            //Transformation Matrix for the TCP inside of the global coordinate system
                                                                                                                                                    //Calculation of wrist center point
      std::array<double, 3> wcp;
      TCP.output();
      wcp[0] = _pos->get_X()*1000-(215*TCP.get_element(0,2));
      wcp[1] = _pos->get_Y()*1000-(215*TCP.get_element(1,2));
      wcp[2] = _pos->get_Z()*1000-(215*TCP.get_element(2,2));


    for (int i = 0; i < 3; ++i) {
        std::cout << "wcp :" << wcp[i] << std::endl;
    }

    if(wcp[0]>0 && wcp[1]>=0)
    {
        cout << "Fall 1" << endl;
        phi1= ((-1)*atan(wcp[1]/wcp[0]))*180/M_PI;
        cout << "phi1: " << phi1 << endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if(-185 < phi1 && phi1 < -175 || 185 > phi1 && phi1 > 175 || -5 < phi1 && phi1 < 5){
          inv_checktheta(phi1,d1, wcp);
        }
        else {
          solution_standard = inv_standardcase(phi1, d1, wcp);

            //calculation of R03
            TMatrix mat1(0, 180, 0, 645);
            TMatrix mat2((0 + sol_standard[0]), 90, 330, 0);
            TMatrix mat3((0 + sol_standard[1]), 0, 1150, 0);
            TMatrix mat4((-90 + sol_standard[2]), 90, 115, 0);

            //Having the transformationmatrix
            TMatrix T03;
            T03 = mat1*mat2*mat3*mat4;
            //Calculating Euler Angles for R03
            std::array<double, 3> euler;
            euler = T03.convertToEulerAngles();

            //Calculating R03
            double _Trans03[6];
            _Trans03[0] = 0;
            _Trans03[1] = 0;
            _Trans03[2] = 0;
            _Trans03[3] = euler[0];
            _Trans03[4] = euler[1];
            _Trans03[5] = euler[2];

            //Transposed matrix of R03
            TMatrix R03(_Trans03);

            TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);

            TMatrix R36 = R03*R06;
            std::array<double, 10> solution_standard_4_5_6;
            solution_standard_4_5_6 = inv_gettheta4_5_6(R36);

            vector<Configuration*>* solutions = new vector<Configuration*>();
            //soltions for positiv theta 5
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));

            //solutions for negative theta 5

            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));

            return solutions;

        }

    }

    else if(wcp[0]<0 && wcp[1]<=0)
    {
        cout << "Fall 2" << endl;
        phi1= 180-atan(wcp[1]/wcp[0])*180/M_PI;
        cout << "phi1: " << phi1 << endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if(-185 < phi1 && phi1 < -175 || 185 > phi1 && phi1 > 175 || -5 < phi1 && phi1 < 5){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
           solution_standard = inv_standardcase(phi1, d1, wcp);
            //calculation of R03
            TMatrix mat1(0, 180, 0, 645);
            TMatrix mat2((0 + sol_standard[0]), 90, 330, 0);
            TMatrix mat3((0 + sol_standard[1]), 0, 1150, 0);
            TMatrix mat4((-90 + sol_standard[2]), 90, 115, 0);

            //Having the transformationmatrix
            TMatrix T03;
            T03 = mat1*mat2*mat3*mat4;
            //Calculating Euler Angles for R03
            std::array<double, 3> euler;
            euler = T03.convertToEulerAngles();

            //Calculating R03
            double _Trans03[6];
            _Trans03[0] = 0;
            _Trans03[1] = 0;
            _Trans03[2] = 0;
            _Trans03[3] = euler[0];
            _Trans03[4] = euler[1];
            _Trans03[5] = euler[2];

            //Transposed matrix of R03
            TMatrix R03(_Trans03);

            TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);

            TMatrix R36 = R03*R06;
            std::array<double, 10> solution_standard_4_5_6;
            solution_standard_4_5_6 = inv_gettheta4_5_6(R36);

            vector<Configuration*>* solutions = new vector<Configuration*>();
            //soltions for positiv theta 5
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));

            //solutions for negative theta 5

            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            return solutions;
        }

    }

    else if(wcp[0]>0 && wcp[1]<=0)
    {
        cout << "Fall 3" << endl;
        phi1=(-1)*atan(wcp[1]/wcp[0])*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if(-185 < phi1 && phi1 < -175 || 185 > phi1 && phi1 > 175 || -5 < phi1 && phi1 < 5){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
            solution_standard = inv_standardcase(phi1, d1, wcp);
            //calculation of R03
            TMatrix mat1(0, 180, 0, 645);
            TMatrix mat2((0 + sol_standard[0]), 90, 330, 0);
            TMatrix mat3((0 + sol_standard[1]), 0, 1150, 0);
            TMatrix mat4((-90 + sol_standard[2]), 90, 115, 0);

            //Having the transformationmatrix
            TMatrix T03;
            T03 = mat1*mat2*mat3*mat4;
            //Calculating Euler Angles for R03
            std::array<double, 3> euler;
            euler = T03.convertToEulerAngles();

            //Calculating R03
            double _Trans03[6];
            _Trans03[0] = 0;
            _Trans03[1] = 0;
            _Trans03[2] = 0;
            _Trans03[3] = euler[0];
            _Trans03[4] = euler[1];
            _Trans03[5] = euler[2];

            //Transposed matrix of R03
            TMatrix R03(_Trans03);

            TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);

            TMatrix R36 = R03*R06;
            std::array<double, 10> solution_standard_4_5_6;
            solution_standard_4_5_6 = inv_gettheta4_5_6(R36);

            vector<Configuration*>* solutions = new vector<Configuration*>();
            //soltions for positiv theta 5
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));

            //solutions for negative theta 5
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            return solutions;
        }
    }

    else if(wcp[0]<0 && wcp[1]>=0)
    {
        cout << "Fall 4" << endl;
        phi1=180-atan(wcp[1]/(-1*wcp[0]))*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if(-185 < phi1 && phi1 < -175 || 185 > phi1 && phi1 > 175 || -5 < phi1 && phi1 < 5){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
            solution_standard = inv_standardcase(phi1, d1, wcp);
            //calculation of R03
            TMatrix mat1(0, 180, 0, 645);
            TMatrix mat2((0 + sol_standard[0]), 90, 330, 0);
            TMatrix mat3((0 + sol_standard[1]), 0, 1150, 0);
            TMatrix mat4((-90 + sol_standard[2]), 90, 115, 0);

            //Having the transformationmatrix
            TMatrix T03;
            T03 = mat1*mat2*mat3*mat4;
            //Calculating Euler Angles for R03
            std::array<double, 3> euler;
            euler = T03.convertToEulerAngles();

            //Calculating R03
            double _Trans03[6];
            _Trans03[0] = 0;
            _Trans03[1] = 0;
            _Trans03[2] = 0;
            _Trans03[3] = euler[0];
            _Trans03[4] = euler[1];
            _Trans03[5] = euler[2];

            //Transposed matrix of R03
            TMatrix R03(_Trans03);

            TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);

            TMatrix R36 = R03*R06;
            std::array<double, 10> solution_standard_4_5_6;
            solution_standard_4_5_6 = inv_gettheta4_5_6(R36);

            vector<Configuration*>* solutions = new vector<Configuration*>();
            //soltions for positiv theta 5
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[0],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[6]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[1],
                                                    solution_standard_4_5_6[4],solution_standard_4_5_6[7]}));

            //solutions for negative theta 5

            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[2],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[8]}));
            solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],
                                                    solution_standard[2],solution_standard_4_5_6[3],
                                                    solution_standard_4_5_6[5],solution_standard_4_5_6[9]}));
            return solutions;
        }
    }

        //Special cases
        //Special case 1:
    else if(wcp[0]==0 && wcp[1]>0)
    {
        //cout << "needs to be implemented" << endl;
        if(wcp[1] > m){
            double d1 = wcp[1];
            phi1 = -90;
            double dpx = wcp[1] - m;
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = 90;
            dpx = wcp[1]+m;
            phi2_phi3 = inv_backwardcase(dpx,dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);

        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            phi1 = -90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = 90;
            dpx = wcp[1] + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);
        }
    }
        //special case 2:
    else if(wcp[0]==0 && wcp[1]<0)
    {
        //cout << "needs to be implemented" << endl;
        if(wcp[1] > m){
            double d1 = wcp[1];
            phi1 = 90;
            double dpx = wcp[1] - m;
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = -90;
            dpx = wcp[1]+m;
            phi2_phi3 = inv_backwardcase(dpx,dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);

        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            phi1 = 90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits(phi1, phi2_phi3);

            phi1 = -90;
            dpx = wcp[1] + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits(phi1, phi2_phi3);
        }
    }

//    vector<Configuration*>* solutions = new vector<Configuration*>();
//    solutions->push_back(new Configuration({sol_standard[0],sol_standard[1],sol_standard[2],0,0,0}));
//    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
//    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));
//
//    return solutions;
}