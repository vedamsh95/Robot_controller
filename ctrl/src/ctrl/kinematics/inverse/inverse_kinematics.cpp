#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>

double o = 115;
double m = 330;
double n = 645;
double a = 1150;
double b = 1220;


vector<Configuration*>* solution_vec = new vector<Configuration*>();

std::vector<vector<double>> solution;
std::array<double, 4> phi2_phi3;
std::vector<std::vector<double>> solution_standard;
std::vector<std::vector<double>> sol_standard;

std::vector<std::vector<double>> sol_specialcase1_1;
std::vector<std::vector<double>> sol_specialcase1_2;
std::vector<std::vector<double>> sol_specialcase2_1;
std::vector<std::vector<double>> sol_specialcase2_2;
std::vector<Configuration*>* sol_specialcase1_1_vec;
std::vector<Configuration*>* sol_specialcase1_2_vec;

std::vector<vector<double>> sol_phi1special1_1;
std::vector<vector<double>> sol_phi1special1_2;
std::vector<vector<double>> sol_phi1special1_3;

double _Trans03[6];

std::array<double, 10> InvKinematics::inv_gettheta4_5_6(TMatrix R36){

    //components of R36 needed for the calculations of Theta 4, 5, 6:
    double ax = R36.get_element(0,2);
    double ay = R36.get_element(1,2);
    double az = R36.get_element(2,2);
    double sz = R36.get_element(2,1);
    double nz = R36.get_element(2,0);

    std::cout << "ax: " << ax << std::endl;
    std::cout << "ay: " << ay << std::endl;
    std::cout << "az: " << az << std::endl;
    std::cout << "sz: " << sz << std::endl;
    std::cout << "nz: " << nz << std::endl;

    // rounding valules to 0
    bool ax_is_0 = false;
    bool ay_is_0 = false;
    bool nz_is_0 = false;
    bool sz_is_0 = false;
    bool az_is_0 = false;
    if(ax <= 0.000001 && ax >= -0.000001){
        ax = 0;
        ax_is_0 = true;
    }

    if(ay <= 0.000001 && ay >= -0.000001){
        ay = 0;
        ay_is_0 = true;
    }

    if(nz <= 0.000001 && nz>= -0.000001){
        nz = 0;
        nz_is_0 = true;
    }
    if(sz <= 0.000001 && sz>= -0.000001){
        sz = 0;
        sz_is_0 = true;
    }
    if(az <= 0.000001 && az>= -0.000001){
        az = 0;
        az_is_0 = true;
    }
    std::cout << "R36" << std::endl;
    R36.output();
    std::cout << "testrechnung: " << atan2(0,0) << endl;
    //Calculations for the varieties of thetas
    array<double, 4> theta4{};
    array<double, 2> theta5{};
    array<double, 4> theta6{};

    //For theta 5 > 0
    //handle error cases of atan2 for ay, ax
    if(ay_is_0 && !ax_is_0){
        theta4[0] = atan2(ay,-ax) * 180/M_PI;
    }else if(!ay_is_0 && ax_is_0){
        theta4[0] = atan2(-ay,ax) * 180/M_PI;
    }else if(ay_is_0 && ax_is_0){
        theta4[0] = atan2(ay,ax) * 180/M_PI;
    }else{
        theta4[0] = atan2(-ay,-ax) * 180/M_PI;
    }
    theta4[1] = theta4[0] + 360;
    //handle error cases of atan2 for az
    if(az_is_0){
        theta5[0] = atan2(sqrt(1-pow((az),2)), az)*180/M_PI;
    }else{
        theta5[0] = atan2(sqrt(1-pow((az),2)), (-1)*az)*180/M_PI;
    }
    theta6[0] = atan2(sz,nz)*180/M_PI;
    theta6[1] = theta6[0] + 360;

    //For theta 5 < 0
    theta4[2] = atan2(ay,ax)*180/M_PI;
    theta4[3] = theta4[2] - 360;
    //handle error cases of atan2 for az
    if(az_is_0){
        theta5[1] = atan2(sqrt(1-pow((az),2)), az)*180/M_PI;
    }else{
        theta5[1] = atan2(sqrt(1-pow((az),2)), (-1)*az)*180/M_PI;
    }
    //handle error cases of atan2 for sz, nz
    if(sz_is_0 && !nz_is_0){
        theta6[2] = atan2(sz, (-1)*nz)*180/M_PI;
    }else if(!sz_is_0 && nz_is_0){
        theta6[2] = atan2((-1)*sz, nz)*180/M_PI;
    }else if(sz_is_0 && nz_is_0){
        theta6[2] = atan2(sz, nz)*180/M_PI;
    }else{
        theta6[2] = atan2((-1)*sz, (-1)*nz)*180/M_PI;
    }
    theta6[3] = theta6[2] - 360;

    std::array<double,10> theta4_5_6 = {theta4[0],theta4[1],theta4[2],theta4[3],
                                        theta5[0],theta5[1],
                                        theta6[0],theta6[1],theta6[2],theta6[3]};

    return theta4_5_6;
}

std::vector<std::vector<double>> InvKinematics::inv_checktheta(double phi1, double d1, std::array<double, 3> wcp) {
    if(-185 < phi1 && phi1 < -175) {
        //cout << "needs to be implemented" << endl;
        double dpx = d1 - m;
        double dpy = wcp[2] - n;

        if(d1 > m && wcp[2] > n){
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 + 360;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 + 180;
            dpx = d1 + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            std::vector<vector<double>> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
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
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 + 360;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 + 180;
            dpx = d1 + m;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            std::vector<std::vector<double>> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
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
            sol_phi1special1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 - 360;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 - 180;
            dpx = d1 + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            std::vector<std::vector<double>> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
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
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = phi1 - 180;
            dpx = d1 + m;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_phi1special1_3 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            std::vector<std::vector<double>> sol_phi1specialcases{sol_phi1special1_1[0], sol_phi1special1_1[1],
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

std::vector<std::vector<double>> InvKinematics::inv_standardcase(double phi1, double d1, std::array<double, 3> wcp) {
    double id = 0;
   //standard cases
    if ((d1 > m) && (-175 < phi1 && phi1 < 175))                                                                               //calculating all the forward cases
    {
        std::cout << "forward case " << std::endl;
        double dpx = d1 - m;                                                                                            //calculating the x and y components of the
        cout << "dpx: " << dpx << endl;
        double dpy = wcp[2] - n;
        cout << "dpy: " << dpy << endl;

        phi2_phi3 = inv_forwardcase(dpx, dpy);
        sol_standard = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
        id = sol_standard.at(0).at(0);
        if(id == 2 || id == 3){
            sol_standard.at(0).at(0) = 100+id;
        }

        else if(id == 11){
            sol_standard.at(0).at(0) = 100+id;
            sol_standard.at(1).at(0) = 100+12;
        }

        else if(id == 4){
            std::cout << "No possible configuration for the robot" << endl;
        }
        return sol_standard;
    }                                                         //distance between the second joint and wcp

    else if ((d1 < m) && (-175 < phi1 && phi1< 175))                     //calculating all the backward cases
    {
        std::cout << "backward case " << std::endl;
        double dpx = m - d1;                                  //calculating the x and y components of the
        std::cout << "dpx: " << dpx << std::endl;
        double dpy = wcp[2] - n;                              //distnance between the second joint and wcp
        std::cout << "dpy: " << dpy << std::endl;

        phi2_phi3 = inv_backwardcase(dpx, dpy);
        sol_standard = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
        id = sol_standard.at(0).at(0);
        if(id == 2 || id == 3){
            sol_standard.at(0).at(0) = 200+id;
        }

        else if(id == 11){
            sol_standard.at(0).at(0) = 200+id;
            sol_standard.at(1).at(0) = 200+12;
        }

        else if(id == 4){
            std::cout << "No possible configuration for the robot" << endl;
        }
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
    double phi3_b_d = -1*(90-(beta*180/M_PI-asin(b/d2)*180/M_PI));                                                             //double phi3_b_d = -1 * (90 - (beta - asin(b / d2)*180/M_PI));        //for backwards elbow down
    std::cout << "phi3_b_d: " << phi3_b_d << std::endl;

    phi2_phi3 = {phi2_b_u, phi2_b_d, phi3_b_u, phi3_b_d};

    return phi2_phi3;
}

std::vector<vector<double>> InvKinematics::inv_checklimits_theta1_2_3(double phi1, array<double, 4> phi2_phi3){
bool elbowup = false;
bool elbowdown = false;
double id = 0;

    if (-185 < phi1 && phi1 < 185) {
        std::cout << "phi1: " << phi1 << std::endl;
        if (-140 < phi2_phi3[0] && phi2_phi3[0] < -5) {
            std::cout << "phi2: " << phi2_phi3[0] << std::endl;
            if (-120 < phi2_phi3[2] && phi2_phi3[2] < 168) {
                std::cout << "phi3: " << phi2_phi3[2] << std::endl;
                elbowup = true;
            }

        }
        if (-140 < phi2_phi3[1] && phi2_phi3[1]< -5) {
            std::cout << "phi2: " << phi2_phi3[1] << std::endl;
            if (-120 < phi2_phi3[3] && phi2_phi3[3] < 168) {
                std::cout << "phi3: " << phi2_phi3[3] << std::endl;
                elbowdown = true;
            }
        }
    }
    if (elbowup == true && elbowdown == true){
        id = 11;
        std::vector<double> elbow_up_vec;
        elbow_up_vec.push_back(id);
        elbow_up_vec.push_back(phi1);
        elbow_up_vec.push_back(phi2_phi3[0]);
        elbow_up_vec.push_back(phi2_phi3[2]);
        id = 12;
        std::vector<double> elbow_down_vec;
        elbow_down_vec.push_back(id);
        elbow_down_vec.push_back(phi1);
        elbow_down_vec.push_back(phi2_phi3[1]);
        elbow_down_vec.push_back(phi2_phi3[3]);

        std::cout << "Both Configurations are true" << endl;
        solution.push_back(elbow_up_vec);
        solution.push_back(elbow_down_vec);
    }

    else if(elbowup == true && elbowdown == false){
        id = 2;
        std::vector<double> elbow_up_vec;
        elbow_up_vec.push_back(id);
        elbow_up_vec.push_back(phi1);
        elbow_up_vec.push_back(phi2_phi3[0]);
        elbow_up_vec.push_back(phi2_phi3[2]);

        std::cout << "Only Elbow up is a possible configuration" << std::endl;
        solution.push_back(elbow_up_vec);
    }
    else if(elbowup == false && elbowdown == true){
        std::vector<double> elbow_down_vec;
        id = 3;
        elbow_down_vec.push_back(id);
        elbow_down_vec.push_back(phi1);
        elbow_down_vec.push_back(phi2_phi3[1]);
        elbow_down_vec.push_back(phi2_phi3[3]);

        std::cout << "Both Configurations are true" << endl;
        solution.push_back(elbow_down_vec);
    }
    else if(elbowup == false && elbowdown == false){
        std::vector<double> no_solution;
        id = 4;
        no_solution.push_back(id);
        std::cout << "No Configuration is reachable" << endl;

        solution.push_back(no_solution);
    }
    return solution;
}

std::vector<Configuration*>* InvKinematics::inv_add_case_to_vec(double phi1, double d1, std::array<double, 3> wcp, SixDPos* _pos, vector<vector<double>> solutions_vec){
    double id = 0;
    vector<Configuration *>* config_vec = new vector<Configuration *>();
    Configuration *upward_config;
    Configuration *downward_config;
    Configuration *single_config;
    //solution_standard = inv_standardcase(phi1, d1, wcp);
    solution_standard = solutions_vec;
    std::cout << solution_standard.at(0).at(1);
    id = solution_standard.at(0).at(0);
    std::cout << id << endl;

    if (id == 111 || id == 211) {

        //Starting calculation of theta 4,5,6 for upwards case
        TMatrix mat1_1(0, 180, 0, 645);
        TMatrix mat2_1((0 + solution_standard.at(0).at(1)), 90, 330, 0);
        TMatrix mat3_1((0 + solution_standard.at(0).at(2)), 0, 1150, 0);
        TMatrix mat4_1((-90 + solution_standard.at(0).at(3)), 90, 115, 0);

        //Starting calculation of theta 4,5,6 for backwards case
        TMatrix mat1_2(0, 180, 0, 645);
        TMatrix mat2_2((0 + solution_standard.at(1).at(1)), 90, 330, 0);
        TMatrix mat3_2((0 + solution_standard.at(1).at(2)), 0, 1150, 0);
        TMatrix mat4_2((-90 + solution_standard.at(1).at(3)), 90, 115, 0);

        TMatrix T03_1;
        TMatrix T03_2;
        T03_1 = mat1_1*mat2_1*mat3_1*mat4_1;
        T03_2 = mat1_2*mat2_2*mat3_2*mat4_2;

        std::vector<std::vector<double>> sol_theta_4_5_6_T03_1 = inv_vec_sol_theta4_5_6(T03_1, _pos);
        std::vector<std::vector<double>> sol_theta_4_5_6_T03_2 = inv_vec_sol_theta4_5_6(T03_2, _pos);
        //get the number of configurations

        double num_config1 = sol_theta_4_5_6_T03_1.at(0).at(0);
        double num_config2 = sol_theta_4_5_6_T03_1.at(0).at(0);

        for (int i = 1; i <= num_config1; i++){
            //Case: elbow down and elbow up forward or backward
            //Transfer the angles to the GUI in radian
            upward_config = new Configuration({solution_standard.at(0).at(1) * M_PI / 180,
                                               solution_standard.at(0).at(2) * M_PI / 180,
                                               solution_standard.at(0).at(3) * M_PI / 180,
                                               sol_theta_4_5_6_T03_1.at(i).at(0)*M_PI/180,
                                               sol_theta_4_5_6_T03_1.at(i).at(1)*M_PI/180,
                                               sol_theta_4_5_6_T03_1.at(i).at(2)*M_PI/180});
            config_vec->push_back(upward_config);

            downward_config = new Configuration({solution_standard.at(0).at(1) * M_PI / 180,
                                               solution_standard.at(0).at(2) * M_PI / 180,
                                               solution_standard.at(0).at(3) * M_PI / 180,
                                               sol_theta_4_5_6_T03_1.at(i).at(0)*M_PI/180,
                                               sol_theta_4_5_6_T03_1.at(i).at(1)*M_PI/180,
                                               sol_theta_4_5_6_T03_1.at(i).at(2)*M_PI/180});
            config_vec->push_back(downward_config);
        }
    }

    else if (id == 102 || id == 103 || id == 202 || id == 203) {

        //Starting calculation of theta 4,5,6 for either upward or backward
        TMatrix mat1(0, 180, 0, 645);
        TMatrix mat2((0 + solution_standard.at(0).at(1)), 90, 330, 0);
        TMatrix mat3((0 + solution_standard.at(0).at(2)), 0, 1150, 0);
        TMatrix mat4((-90 + solution_standard.at(0).at(3)), 90, 115, 0);

        TMatrix T03;
        T03 = mat1*mat2*mat3*mat4;
        T03.output();

        std::vector<std::vector<double>> sol_theta_4_5_6_T03 = inv_vec_sol_theta4_5_6(T03, _pos);
        //get the number of configurations

        double num_config1 = sol_theta_4_5_6_T03.at(0).at(0);

        for (int i = 1; i <= num_config1; i++){

            single_config = new Configuration({solution_standard.at(0).at(1) * M_PI / 180,
                                               solution_standard.at(0).at(2) * M_PI / 180,
                                               solution_standard.at(0).at(3) * M_PI / 180,
                                               sol_theta_4_5_6_T03.at(i).at(0)*M_PI/180,
                                               sol_theta_4_5_6_T03.at(i).at(1)*M_PI/180,
                                               sol_theta_4_5_6_T03.at(i).at(2)*M_PI/180});
            config_vec->push_back(single_config);

            std::cout << "Configuration " << i << ": " << single_config->get_configuration().at(0) << ", "<< single_config->get_configuration().at(1) << ", "
                    << single_config->get_configuration().at(2) << ", "<< single_config->get_configuration().at(3) << ", "
                    << single_config->get_configuration().at(4) << ", "<< single_config->get_configuration().at(5) << endl;
        }

    }
    else if (id == 104 || id == 204) {
        std::cout << id << " is Not a possible configuration!" << std::endl;
    }
    return config_vec;
}

std::vector<std::vector<double>> InvKinematics::inv_vec_sol_theta4_5_6(TMatrix T03, SixDPos* _pos){

    std::array<double, 3> euler{};
    euler = T03.convertToEulerAngles();

    //Calculating R03
    double _Trans03[6];
    _Trans03[0] = 0;
    _Trans03[1] = 0;
    _Trans03[2] = 0;
    _Trans03[3] = euler[2];
    _Trans03[4] = euler[1];
    _Trans03[5] = euler[0];

    //Transposed matrix of R03
    TMatrix R03(_Trans03);
    std::cout << "R03: " << std::endl;
    R03.output();

    TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);
    std::cout << "R06: " << std::endl;
    R06.output();

    TMatrix R36 = R03*R06;
    std::array<double, 10> solution_standard_4_5_6;
    solution_standard_4_5_6 = inv_gettheta4_5_6(R36);
    std::vector<vector<double>> solution_theta_4_5_6 = inv_checklimits_theta4_5_6(solution_standard_4_5_6);

    return solution_theta_4_5_6;
}

std::vector<std::vector<double>> InvKinematics::inv_checklimits_theta4_5_6(std::array<double, 10> solution_standard_4_5_6){

    double id = 0;
    std::vector<double> config1;
    std::vector<double> config2;
    std::vector<double> config3;
    std::vector<double> config4;
    std::vector<double> config5;
    std::vector<double> config6;
    std::vector<double> config7;
    std::vector<double> config8;
    std::vector<double> vec_id;
    std::vector<vector<double>> sol_theta_4_5_6;
    vec_id.push_back(id);
    sol_theta_4_5_6.push_back(vec_id);
    //filtering case of equal configurations for theta 4, 5, 6, especially for 0, 0 ,0
    if((solution_standard_4_5_6[0] == solution_standard_4_5_6[2]) &&
    (solution_standard_4_5_6[4] == solution_standard_4_5_6[5]) &&
    (solution_standard_4_5_6[6] == solution_standard_4_5_6[8])){
        //declaring the second theta 4,5,6 values out of range (otherwise there are two equal configurations)
        solution_standard_4_5_6[2] = 1000;
        solution_standard_4_5_6[5] = 1000;
        solution_standard_4_5_6[8] = 1000;
    }
    if(-125 <= solution_standard_4_5_6[4] && solution_standard_4_5_6[4] <=  125){
        if(-350 <= solution_standard_4_5_6[0] && solution_standard_4_5_6[0] <= 350){
            if(-350 <= solution_standard_4_5_6[6] && solution_standard_4_5_6[6] <= 350){
                //Config 1
                id++;
                config1.push_back(solution_standard_4_5_6[0]);
                config1.push_back(solution_standard_4_5_6[4]);
                config1.push_back(solution_standard_4_5_6[6]);
                sol_theta_4_5_6.push_back(config1);
            }
            if(-350 <= solution_standard_4_5_6[7] && solution_standard_4_5_6[7] <= 350){
                //Config 2
                id++;
                config2.push_back(solution_standard_4_5_6[0]);
                config2.push_back(solution_standard_4_5_6[4]);
                config2.push_back(solution_standard_4_5_6[7]);
                sol_theta_4_5_6.push_back(config2);
            }
        }
        if(-350 <= solution_standard_4_5_6[1] && solution_standard_4_5_6[1] <= 350){
            if(-350 <= solution_standard_4_5_6[6] && solution_standard_4_5_6[6] <= 350){
                //Config 3
                id++;
                config3.push_back(solution_standard_4_5_6[1]);
                config3.push_back(solution_standard_4_5_6[4]);
                config3.push_back(solution_standard_4_5_6[6]);
                sol_theta_4_5_6.push_back(config3);
            }
            if(-350 <= solution_standard_4_5_6[7] && solution_standard_4_5_6[7] <= 350){
                //Config 4
                id++;
                config4.push_back(solution_standard_4_5_6[1]);
                config4.push_back(solution_standard_4_5_6[4]);
                config4.push_back(solution_standard_4_5_6[7]);
                sol_theta_4_5_6.push_back(config4);
            }
        }
    }

    if(-125 <= solution_standard_4_5_6[5] && solution_standard_4_5_6[5]<= 125){
        if(-350 <= solution_standard_4_5_6[2] && solution_standard_4_5_6[2] <= 350){
            if(-350 <= solution_standard_4_5_6[8] && solution_standard_4_5_6[8] <= 350){
                //Config 5
                id++;
                config5.push_back(solution_standard_4_5_6[2]);
                config5.push_back(solution_standard_4_5_6[5]);
                config5.push_back(solution_standard_4_5_6[8]);
                sol_theta_4_5_6.push_back(config5);
            }
            if(-350 <= solution_standard_4_5_6[9] && solution_standard_4_5_6[9] <= 350){
                //Config 6
                id++;
                config6.push_back(solution_standard_4_5_6[2]);
                config6.push_back(solution_standard_4_5_6[5]);
                config6.push_back(solution_standard_4_5_6[9]);
                sol_theta_4_5_6.push_back(config6);
            }
        }
        if(-350 <= solution_standard_4_5_6[3] && solution_standard_4_5_6[3] <= 350){
            if(-350 <= solution_standard_4_5_6[8] && solution_standard_4_5_6[8]<= 350){
                //Config 7
                id++;
                config7.push_back(solution_standard_4_5_6[3]);
                config7.push_back(solution_standard_4_5_6[5]);
                config7.push_back(solution_standard_4_5_6[8]);
                sol_theta_4_5_6.push_back(config7);
            }
            if(-350 <= solution_standard_4_5_6[9] && solution_standard_4_5_6[9] <= 350){
                //Config 8
                id++;
                config8.push_back(solution_standard_4_5_6[3]);
                config8.push_back(solution_standard_4_5_6[5]);
                config8.push_back(solution_standard_4_5_6[9]);
                sol_theta_4_5_6.push_back(config8);
            }
        }
    }

    sol_theta_4_5_6.at(0).at(0) = id;
    std::cout << "id: " << id << std::endl;
    return sol_theta_4_5_6;
}


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    //you should call your inverse kinematics functions here!

    double phi1;
    double id = 0;
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
      if(_pos->get_X() > 0) {
          wcp[0] = _pos->get_X() * 1000 - (215 * TCP.get_element(0, 2));
          wcp[1] = _pos->get_Y() * 1000 - (215 * TCP.get_element(1, 2));
          wcp[2] = _pos->get_Z() * 1000 - (215 * TCP.get_element(2, 2));
      }
      else{
          wcp[0] = _pos->get_X() * 1000 + (215 * TCP.get_element(0, 2));
          wcp[1] = _pos->get_Y() * 1000 - (215 * TCP.get_element(1, 2));
          wcp[2] = _pos->get_Z() * 1000 - (215 * TCP.get_element(2, 2));
      }

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
                //solution_vec contains the possible configurations for a standardcase for all joints
                solution_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, solution_standard);
        }

    }

    else if(wcp[0]<0 && wcp[1]<=0)
    {
        cout << "Fall 2" << endl;
        phi1= 180-atan(wcp[1]/wcp[0])*180/M_PI;
        cout << "phi1: " << phi1 << endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < phi1 && phi1 < -175) || (185 > phi1 && phi1 > 175) || (-5 < phi1 && phi1 < 5)){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
                //solution_vec contains the possible configurations for a standardcase for all joints
                solution_standard = inv_standardcase(phi1, d1, wcp);
                solution_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, solution_standard);
        }

    }

    else if(wcp[0]>0 && wcp[1]<=0)
    {
        cout << "Fall 3" << endl;
        phi1=(-1)*atan(wcp[1]/wcp[0])*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < phi1 && phi1 < -175) || (185 > phi1 && phi1 > 175) || (-5 < phi1 && phi1 < 5)){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
                //solution_vec contains the possible configurations for a standardcase for all joints
                solution_standard = inv_standardcase(phi1, d1, wcp);
                solution_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, solution_standard);
        }
    }

    else if(wcp[0]<0 && wcp[1]>=0)
    {
        cout << "Fall 4" << endl;
        phi1=180-atan(wcp[1]/(-1*wcp[0]))*180/M_PI;
        std::cout << "phi1: " << phi1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < phi1 && phi1 < -175) || (185 > phi1 && phi1 > 175) || (-5 < phi1 && phi1 < 5)){
            inv_checktheta(phi1,d1, wcp);
        }
        else {
                //solution_vec contains the possible configurations for a standardcase for all joints
                solution_standard = inv_standardcase(phi1, d1, wcp);
                solution_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, solution_standard);
        }
    }

        //Special cases
        //Special case 1:
    else if((wcp[0] <= 0.001 && wcp[0] >= -0.001) && wcp[1] > 0)
    {
        //cout << "needs to be implemented" << endl;
        if(wcp[1] > m){
            double d1 = wcp[1];
            phi1 = -90;
            double dpx = wcp[1] - m;
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_forwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
            sol_specialcase1_1_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, sol_specialcase1_1);
            double size1_1 = sol_specialcase1_1_vec->size();
          //  for(int i = 0; i<=size1_1; i++){
           //     solution_vec->push_back(sol_specialcase1_1_vec->at(i)->get_configuration());
           // }

            phi1 = 90;
            dpx = wcp[1]+m;
            phi2_phi3 = inv_backwardcase(dpx,dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
            sol_specialcase1_2_vec = inv_add_case_to_vec(phi1, d1, wcp, _pos, sol_specialcase1_2);
            double size1_2 = sol_specialcase1_2_vec->size();

        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            phi1 = -90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = 90;
            dpx = wcp[1] + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
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
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = -90;
            dpx = wcp[1]+m;
            phi2_phi3 = inv_backwardcase(dpx,dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            phi1 = 90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);

            phi1 = -90;
            dpx = wcp[1] + m;
            phi2_phi3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(phi1, phi2_phi3);
        }
    }

    //vector<Configuration*>* solutions = new vector<Configuration*>();
/*  //solutions->push_back(new Configuration({solution_standard[0],solution_standard[1],solution_standard[2],0,0,0}));
    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));*/

    //return solutions;
    return solution_vec;
}