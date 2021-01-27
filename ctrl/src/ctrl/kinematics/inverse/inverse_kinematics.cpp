#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <cmath>
#include <iostream>
#include "ConfigProvider.h"
#include "Singularity.h"

/* Calculating the transposed Matrix for R03
 * @requires: this TMatrix RotationMatrix 4x4 of R03
 * @ensures:  transposed TMatrix 4x4 R03
 * */
TMatrix InvKinematics::transposematrix(TMatrix T03){
    TMatrix T03_inv;
    std::array<double, 16> T03_arr;

    for(int h = 0; h < 4; h++) {
        for(int w = 0; w < 4; w++) {
            if (h == 0 && w == 0) {
                T03_arr[0] = T03.get_element(0,0);
            } else if (h == 0 && w == 1) {
                T03_arr[1] = T03.get_element(1,0);
            } else if (h == 0 && w == 2) {
                T03_arr[2] = T03.get_element(2,0);
            } else if (h == 0 && w == 3) {
                T03_arr[3] = T03.get_element(3,0);
            } else if (h == 1 && w == 0) {
                T03_arr[4] = T03.get_element(0,1);
            } else if (h == 1 && w == 1) {
                T03_arr[5] = T03.get_element(1,1);
            } else if (h == 1 && w == 2) {
                T03_arr[6] = T03.get_element(2,1);
            } else if (h == 1 && w == 3) {
                T03_arr[7] = T03.get_element(3,1);
            } else if (h == 2 && w == 0) {
                T03_arr[8] = T03.get_element(0,2);
            } else if (h == 2 && w == 1) {
                T03_arr[9] = T03.get_element(1,2);
            } else if (h == 2 && w == 2) {
                T03_arr[10] = T03.get_element(2,2);
            } else if (h == 2 && w == 3) {
                T03_arr[11] = T03.get_element(3,2);
            } else if (h == 3 && w == 0) {
                T03_arr[12] = 0;
            } else if (h == 3 && w == 1) {
                T03_arr[13] = 0;
            } else if (h == 3 && w == 2) {
                T03_arr[14] = 0;
            } else if (h == 3 && w == 3) {
                T03_arr[15] = T03.get_element(3,3);
            }
        }
    }
    T03_inv = (T03_arr);
    return T03_inv;
}

/* Calculating Theta 4, 5 and 6
 * @requires: this TMatrix RotationMatrix 4x4 of R36
 * @ensures:  double array[10] with different values for theta 4, theta 5, and theta 6 in degree
 * */
 std::array<double, 10> InvKinematics::inv_gettheta4_5_6(TMatrix R36){

    Singularity inv_singularity;

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
    if(az <= 1.000001 && az>= 0.999999){
        az = 1;
    }
    if(az >= -1.000001 && az <= -0.999999){
        az = -1;
    }

    //Calculations for the varieties of thetas
    array<double, 4> theta4{};
    array<double, 2> theta5{};
    array<double, 4> theta6{};
    array<double, 2> sing_theta4_6{};

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
    if(theta4[0] > 0) {
        theta4[1] = theta4[0] - 360;
    }
    else{
        theta4[1] = theta4[0] + 360;
    }
    //handle error cases of atan2 for az
    if(az_is_0){
        theta5[0] = atan2(sqrt(1-pow((az),2)), az)*180/M_PI;
    }else{
        theta5[0] = atan2(sqrt(1-pow((az),2)), (-1)*az)*180/M_PI;
    }
    theta6[0] = atan2(sz,nz)*180/M_PI;
    if(theta6[0]> 0){
        theta6[1] = theta6[0] - 360;
    }
    else{
        theta6[1] = theta6[0] + 360;
    }

    //For theta 5 < 0
    theta4[2] = atan2(ay,ax)*180/M_PI;
    if(theta4[2] > 0) {
        theta4[3] = theta4[2] - 360;
    }
    else{
        theta4[3] = theta4[2] + 360;
    }
    //handle error cases of atan2 for az
    if(az_is_0){
        theta5[1] = atan2(sqrt(1-pow((az),2)), az)*180/M_PI;
    }else{
        theta5[1] = atan2((-1)*sqrt(1-pow((az),2)), (-1)*az)*180/M_PI;
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

    if(theta6[2] >= 0) {
        theta6[3] = theta6[2] - 360;
    }
    else{
        theta6[3] = theta6[2] + 360;
    }

    //check for wrist singularity for theta 5
    if((theta5[0] <= 0 + margin_point && theta5[0] >= 0 - margin_point) || (theta5[1] <= 0 + margin_point && theta5[1] >= 0 - margin_point)){
        std::cout << "There is a Wrist Singularity." << std::endl;
        sing_theta4_6 = inv_singularity.wrist_singularity(theta4[0], theta6[0]);

        theta4_5_6 = {sing_theta4_6[0],sing_theta4_6[0],sing_theta4_6[0],sing_theta4_6[0],
                      theta5[0],theta5[1],
                      sing_theta4_6[1],sing_theta4_6[1],sing_theta4_6[1],sing_theta4_6[1]};

    }else{
        theta4_5_6 = {theta4[0],theta4[1],theta4[2],theta4[3],
                      theta5[0],theta5[1],
                      theta6[0],theta6[1],theta6[2],theta6[3]};
    }
    std::cout << "Theta 4: " << theta4_5_6[0] << std::endl;
    std::cout << "Theta 4: " << theta4_5_6[1] << std::endl;
    std::cout << "Theta 4: " << theta4_5_6[2] << std::endl;
    std::cout << "Theta 4: " << theta4_5_6[3] << std::endl;
    std::cout << "Theta 5: " << theta4_5_6[4] << std::endl;
    std::cout << "Theta 5: " << theta4_5_6[5] << std::endl;
    std::cout << "Theta 6: " << theta4_5_6[6] << std::endl;
    std::cout << "Theta 6: " << theta4_5_6[7] << std::endl;
    std::cout << "Theta 6: " << theta4_5_6[8] << std::endl;
    std::cout << "Theta 6: " << theta4_5_6[9] << std::endl;
    return theta4_5_6;
}

/* Calculating the special cases for Theta 1
 * @requires:   Theta1  :   Rotation of the first joint in degree
 *              d1      :   distance between the rootjoint and the wcp in mm
 *              wcp     :   Position of the robots wrist center point (x,y,z)
 *              _pos    :   Input values of the user containing position and orientation of the end position
 * @ensures:    sol_theta1_specialcases_vec :   a configuration vector with all possible configurations of the joints
 * */
std::vector<Configuration*>* InvKinematics::inv_checktheta(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos) {
    std::vector<Configuration*>* sol_theta1_specialcases_vec = new vector<Configuration*>();
    double id = 0;
    if(-185 < theta1 && theta1 < -175) {
        std::cout << "theta1: (-185 < theta1 && theta1 < -175)" << std::endl;
        //initialize dpx and dpy (as needed in first if statement)
        double dpx = d1 - m;
        double dpy = wcp[2] - n;

        if(d1 > m && wcp[2] > n){

            //step1
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
                sol_theta1_special1_1.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1 << ")" << std:: endl;
            }

            //step2
            double theta1_2 = theta1 + 180;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1_2, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
                sol_theta1_special1_2.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_2 << ")" << std:: endl;
            }

            //step3
            double theta1_3 = theta1 + 180;
            dpx = d1 + m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1_3, theta2_theta3);
            id = sol_theta1_special1_3.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
                sol_theta1_special1_3.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_3 << ")" << std:: endl;
            }
        }

        else if(d1 < m){
            dpx = m - d1;
            dpy = wcp[2] - n;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
                sol_theta1_special1_1.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1 << ")" << std:: endl;
            }

            double theta1_2 = theta1 + 180;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_2 << ")" << std:: endl;
            }


            double theta1_3 = theta1 + 180;
            dpx = d1 + m;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_3.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
                sol_theta1_special1_3.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_3 << ")" << std:: endl;
            }

        }
    }
    else if (185 > theta1 && theta1 > 175) {
        std::cout << "theta1: (185 > theta1 && theta1 > 175)" << std::endl;
        double dpx = d1 - m;
        double dpy = wcp[2] - n;

        if(d1 > m && wcp[2] > n){
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
                sol_theta1_special1_1.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1 << ")" << std:: endl;
            }


            double theta1_2 = theta1 - 180;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1_2, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
                sol_theta1_special1_2.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_2 << ")" << std:: endl;
            }


            double theta1_3 = theta1 - 180;
            dpx = d1 + m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1_3, theta2_theta3);
            id = sol_theta1_special1_3.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
                sol_theta1_special1_3.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_3 << ")" << std:: endl;
            }

        }

        if(d1 < m){
            dpx = m - d1;
            dpy = wcp[2] - n;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
                sol_theta1_special1_1.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1 << ")" << std:: endl;
            }

            double theta1_2 = theta1 - 180;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1_2, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_2 << ")" << std:: endl;
            }



            double theta1_3 = theta1 - 180;
            dpx = d1 + m;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1_3, theta2_theta3);
            id = sol_theta1_special1_3.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
                sol_theta1_special1_3.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_3 << ")" << std:: endl;
            }
        }
    }
    else if (-5 < theta1 && 5 > theta1) {
        double dpx = d1 - m;
        double dpy = wcp[2] - n;
        std::cout << "Theta 1: -5 < theta1 && 5 > theta1." << std::endl;
        if(d1 > m && wcp[2] > n){
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
                sol_theta1_special1_1.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = 0;
            if(sol_specialcase1_1_vec != NULL){
                size1_1 = sol_specialcase1_1_vec->size();
            }

            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1 << ")" << std:: endl;
            }


            double theta1_2 = theta1 - 180;
            dpx = d1 + m;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1_2, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_2 << ")" << std:: endl;
            }


            double theta1_3 = theta1 + 180;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1_3, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 200+id;
                sol_theta1_special1_3.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 > m && z > n) (theta: " << theta1_3 << ")" << std:: endl;
            }

        }

        if(d1 < m){
            dpx = m - d1;
            dpy = wcp[2] - n;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_theta1_special1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
                sol_theta1_special1_1.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_theta1_special1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1 << ")" << std:: endl;
            }

            double theta1_2 = theta1 - 180;
            dpx = d1 + m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_2 = inv_checklimits_theta1_2_3(theta1_2, theta2_theta3);
            id = sol_theta1_special1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 100+id;
                sol_theta1_special1_2.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1_2, d1, wcp, _pos, sol_theta1_special1_2);
            // size1_1 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_2 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_2 << ")" << std:: endl;
            }



            double theta1_3 = theta1 + 180;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_theta1_special1_3 = inv_checklimits_theta1_2_3(theta1_3, theta2_theta3);
            id = sol_theta1_special1_3.at(0).at(0);
            //Checking how many configurations there are for Theta 1, Theta 2 and Theta 3
            if(id == 2 || id == 3){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_3.at(0).at(0) = 100+id;
                sol_theta1_special1_3.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_3_vec = inv_add_case_to_vec(theta1_3, d1, wcp, _pos, sol_theta1_special1_3);
            // size1_1 is the number of configurations inside the vector
            double size1_3 = sol_specialcase1_3_vec->size();
            for(int i = 0; i < size1_3; i++){
                // push every configuration into the complete vector
                sol_theta1_specialcases_vec->push_back(sol_specialcase1_3_vec->at(i));
            }
            if(size1_3 == 0){
                std::cout << "No configuration added to sol_theta1_specialcases_vec for (d1 < m) (theta: " << theta1_3 << ")" << std:: endl;
            }
        }

    }
    return sol_theta1_specialcases_vec;
}

/* Calculating for the standardcases
 * @requires:   Theta1  :   Rotation of the first joint in degree
 *              d1      :   distnance between the rootjoint and the wcp in mm
 *              wcp     :   Position of the robots wrist center point (x,y,z)
 * @ensures:    sol_standard :   a vector inside a vector that contains all possible solutions for the first three joints
 * */
std::vector<std::vector<double>> InvKinematics::inv_standardcase(double theta1, double d1, std::array<double, 3> wcp) {
    double id = 0;
   //standard cases
    if ((d1 > m) && (-175 < theta1 && theta1 < 175))                                                                               //calculating all the forward cases
    {
        std::cout << "forward case " << std::endl;
        double dpx = d1 - m;                                                                                            //calculating the x and y components of the
        cout << "dpx: " << dpx << endl;
        double dpy = wcp[2] - n;
        cout << "dpy: " << dpy << endl;

        theta2_theta3 = inv_forwardcase(dpx, dpy);
        sol_standard = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
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
    }                                                         //distance between the second joint and wcp

    else if ((d1 < m) && (-175 < theta1 && theta1 < 175))                     //calculating all the backward cases
    {
        std::cout << "backward case " << std::endl;
        double dpx = m - d1;                                  //calculating the x and y components of the
        std::cout << "dpx: " << dpx << std::endl;
        double dpy = wcp[2] - n;                              //distnance between the second joint and wcp
        std::cout << "dpy: " << dpy << std::endl;

        theta2_theta3 = inv_backwardcase(dpx, dpy);
        sol_standard = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
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
    }
    return sol_standard;
}

/* Calculate Theta 2 and 3 for the forward case
 * @requires:   dpx  :   distance on the x-axis from the second joint to the wcp
 *              dpy  :   distance on the y-axis from the second joint to the wcp
 * @ensures:    theta2_theta3 :   double array[4] that contains all solutions for theta 2 and theta 3
 * */
std::array<double, 4> InvKinematics::inv_forwardcase(double dpx, double dpy){

    double d3 = sqrt(dpx * dpx + dpy * dpy);                                                                        //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / ((-1)*2 * a * d2)));
    std::cout << "beta: " <<  beta << std::endl;
    double pre_alpha1 = sin(beta)*(d2/d3);
    std::cout << "pre_alpha1: " << pre_alpha1 << endl;
    double alpha1 = (asin(pre_alpha1)*180/M_PI);
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3))*180/M_PI;
    std::cout << "alpha2: " << alpha2 << std::endl;

    double theta2_f_u = (-1) * (alpha1 + alpha2);                                                                       //for forward elbow up
    cout << "theta2_f_u: " << theta2_f_u << endl;
    double theta2_f_d = (-1) * (alpha2 - alpha1);                                                                       //for forward elbow down
    cout << "theta2_f_d: " << theta2_f_d << endl;

    double theta3_f_u = 360 - (beta * 180 / M_PI) - (asin(b / d2) * 180 / M_PI) - 90;                                                            //for forward elbow up
    cout << "theta3_f_u: " << theta3_f_u << endl;
    double theta3_f_d = (beta * 180 / M_PI) - ((asin(b / d2)) * 180 / M_PI) - 90;                                                                  //for forward elbow down
    cout << "theta3_f_d: " << theta3_f_d << endl;

    theta2_theta3 = {theta2_f_u, theta2_f_d, theta3_f_u, theta3_f_d};

    return theta2_theta3;
}

/* Calculate Theta 2 and 3 for the backward case
 * @requires:   dpx  :   distance on the x-axis from the second joint to the wcp
 *              dpy  :   distance on the y-axis from the second joint to the wcp
 * @ensures:    theta2_theta3 :   double array[4] that contains all solutions for theta 2 and theta 3
 * */
std::array<double, 4> InvKinematics::inv_backwardcase(double dpx, double dpy) {
    double d3 = sqrt(dpx * dpx + dpy * dpy);                   //direct distance between joint and wcp
    std::cout << "d3: " << d3 << std::endl;
    double d2 = sqrt(o * o + b * b);
    std::cout << "d2: " << d2 << std::endl;
    double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / ((-1)*2 * a * d2)));
    std::cout << "beta: " <<  beta << std::endl;
    double alpha1 = (asin(sin(beta) * (d2 / d3)))*180/M_PI;
    std::cout << "alpha1: " << alpha1 << std::endl;
    double alpha2 = (asin(dpy / d3))*180/M_PI;
    std::cout << "alpha2: " << alpha2 << std::endl;

    double theta2_b_u = (-1) * (180 - (alpha1 + alpha2));          //for backwards elbow up
    std::cout << "theta2_b_u: " << theta2_b_u << std::endl;
    double theta2_b_d = (-1) * (180 - (alpha2 - alpha1));          //for backwards elbow down
    std::cout << "theta2_b_d: " << theta2_b_d << std::endl;

    double theta3_b_u = (-1) * (90 - (beta * 180 / M_PI - asin(b / d2) * 180 / M_PI));              //for backwards elbow up
    std::cout << "theta3_b_u: " << theta3_b_u << std::endl;
    double theta3_b_d = 270 - beta * 180 / M_PI - (asin(b / d2) * 180 / M_PI);                                                             //double theta3_b_d = -1 * (90 - (beta - asin(b / d2)*180/M_PI));        //for backwards elbow down
    std::cout << "theta3_b_d: " << theta3_b_d << std::endl;

    theta2_theta3 = {theta2_b_u, theta2_b_d, theta3_b_u, theta3_b_d};

    return theta2_theta3;
}

/* Checking if Theta 1, 2 and 3 are in range of the boundaries
 * @requires:   theta1  :           Rotation of the first joint in degree
 *              theta2_theta3 :     double array[4] that contains all solutions for theta 2 and theta 3
 * @ensures:    solution    :       Vector inside a vector that contains all solutions for theta 1, 2, 3 that are inside
 *                                  the boundaries
 * */
std::vector<vector<double>> InvKinematics::inv_checklimits_theta1_2_3(double theta1, array<double, 4> theta2_theta3){
bool elbowup = false;
bool elbowdown = false;
double id = 0;
std::vector<double> elbow_up_vec;
std::vector<double> elbow_down_vec;
elbow_down_vec.clear();
elbow_up_vec.clear();
solution.clear();

    if (theta1_l <= theta1 && theta1 <= theta1_u) {
        std::cout << "theta1: " << theta1 << std::endl;
        if (theta2_l <= theta2_theta3[0] && theta2_theta3[0] < theta2_u) {
            std::cout << "theta2: " << theta2_theta3[0] << std::endl;
             if (theta3_l <= theta2_theta3[2] && theta2_theta3[2] < theta3_u) {
                std::cout << "theta3: " << theta2_theta3[2] << std::endl;
                elbowup = true;
            }

        }
        if(theta2_l <= theta2_theta3[1] && theta2_theta3[1] <= theta2_u) {
            std::cout << "theta2: " << theta2_theta3[1] << std::endl;
            if (theta3_l <= theta2_theta3[3] && theta2_theta3[3] <= theta3_u) {
                std::cout << "theta3: " << theta2_theta3[3] << std::endl;
                elbowdown = true;
            }
        }
    }
    if (elbowup == true && elbowdown == true){
        id = 11;
        elbow_up_vec.clear();
        elbow_up_vec.push_back(id);
        elbow_up_vec.push_back(theta1);
        elbow_up_vec.push_back(theta2_theta3[0]);
        elbow_up_vec.push_back(theta2_theta3[2]);
        id = 12;
        elbow_down_vec.push_back(id);
        elbow_down_vec.push_back(theta1);
        elbow_down_vec.push_back(theta2_theta3[1]);
        elbow_down_vec.push_back(theta2_theta3[3]);

        std::cout << "Both Configurations are true" << endl;
        solution.push_back(elbow_up_vec);
        solution.push_back(elbow_down_vec);
    }

    else if(elbowup == true && elbowdown == false){
        id = 2;
        elbow_up_vec.push_back(id);
        elbow_up_vec.push_back(theta1);
        elbow_up_vec.push_back(theta2_theta3[0]);
        elbow_up_vec.push_back(theta2_theta3[2]);

        std::cout << "checklimits: Only Elbow up is a possible configuration" << std::endl;
        solution.push_back(elbow_up_vec);
    }
    else if(elbowup == false && elbowdown == true){
        id = 3;
        elbow_down_vec.push_back(id);
        elbow_down_vec.push_back(theta1);
        elbow_down_vec.push_back(theta2_theta3[1]);
        elbow_down_vec.push_back(theta2_theta3[3]);

        std::cout << "checklimits: Only Elbow Down is a possible Configuration." << endl;
        solution.push_back(elbow_down_vec);
    }
    else if(elbowup == false && elbowdown == false){
        std::vector<double> no_solution;
        id = 4;
        no_solution.push_back(id);
        std::cout << "checklimits: No Configuration is reachable" << endl;

        solution.push_back(no_solution);
    }
    return solution;
}


/* Adding all the possible solutions to a configuration vector
 * @requires:   Theta1  :   Rotation of the first joint in degree
 *              d1      :   distance between the rootjoint and the wcp in mm
 *              wcp     :   double array[3] that contains position of the robots wrist center point (x,y,z)
 *              _pos    :   Input values of the user containing position and orientation of the end position
 *              solutions_vec:  Vector inside a vector that contains solutions for Theta 1, 2, 3
 * @ensures:    config_vec :   a configuration vector with all possible configurations of the joints
 * */
std::vector<Configuration*>* InvKinematics::inv_add_case_to_vec(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos, vector<vector<double>> solutions_vec){
    double id = 0;

    vector<Configuration *>* config_vec = new vector<Configuration *>();
    Configuration *upward_config;
    Configuration *downward_config;
    Configuration *single_config;
    solution_standard = solutions_vec;
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

       TMatrix T03_1_invert = transposematrix(T03_1);
       TMatrix T03_2_invert = transposematrix(T03_2);

        std::vector<std::vector<double>> sol_theta_4_5_6_T03_1 = inv_vec_sol_theta4_5_6(T03_1_invert, _pos);
        std::vector<std::vector<double>> sol_theta_4_5_6_T03_2 = inv_vec_sol_theta4_5_6(T03_2_invert, _pos);
        //get the number of configurations

        double num_config1 = sol_theta_4_5_6_T03_1.at(0).at(0);
        std::cout << "num_config1: " << num_config1 << std::endl;
        double num_config2 = sol_theta_4_5_6_T03_2.at(0).at(0);
        std::cout << "num_config2: " << num_config2 << std::endl;

            for (int i = 1; i <= num_config1; i++) {
                //Case: elbow down and elbow up forward or backward
                //Transfer the angles to the GUI in radian
                upward_config = new Configuration({solution_standard.at(0).at(1) * M_PI / 180,
                                                   solution_standard.at(0).at(2) * M_PI / 180,
                                                   solution_standard.at(0).at(3) * M_PI / 180,
                                                   sol_theta_4_5_6_T03_1.at(i).at(0) * M_PI / 180,
                                                   sol_theta_4_5_6_T03_1.at(i).at(1) * M_PI / 180,
                                                   sol_theta_4_5_6_T03_1.at(i).at(2) * M_PI / 180});
                config_vec->push_back(upward_config);
                std::cout << "Configuration upward: " << i << ": " << upward_config->get_configuration().at(0) << ", "
                          << upward_config->get_configuration().at(1) << ", "
                          << upward_config->get_configuration().at(2) << ", "
                          << upward_config->get_configuration().at(3)
                          << ", "
                          << upward_config->get_configuration().at(4) << ", "
                          << upward_config->get_configuration().at(5)
                          << endl;
            }


            for (int i = 1; i <= num_config2; i++){
                     downward_config = new Configuration({solution_standard.at(1).at(1) * M_PI / 180,
                                                   solution_standard.at(1).at(2) * M_PI / 180,
                                                   solution_standard.at(1).at(3) * M_PI / 180,
                                                   sol_theta_4_5_6_T03_2.at(i).at(0)*M_PI/180,
                                                   sol_theta_4_5_6_T03_2.at(i).at(1)*M_PI/180,
                                                   sol_theta_4_5_6_T03_2.at(i).at(2)*M_PI/180});
                    config_vec->push_back(downward_config);
                    //std::cout << "Test" << std::endl;
                    std::cout << "Configuration downward: " << i << ": " << downward_config->get_configuration().at(0) << ", "
                    << downward_config->get_configuration().at(1) << ", "
                              << downward_config->get_configuration().at(2) << ", "
                              << downward_config->get_configuration().at(3) << ", "
                              << downward_config->get_configuration().at(4) << ", "
                              << downward_config->get_configuration().at(5) << std::endl;
                }

        }


    else if (id == 102 || id == 103 || id == 202 || id == 203) {

        //Starting calculation of theta 4,5,6 for either upward or backward

        TMatrix mat1(0, 180, 0, 645);
        TMatrix mat2((0 + solution_standard.at(0).at(1)), 90, 330, 0);
        TMatrix mat3((0 + solution_standard.at(0).at(2)), 0, 1150, 0);
        TMatrix mat4((-90 + solution_standard.at(0).at(3)), 90, 115, 0);

        TMatrix T03;
        TMatrix T03_invert;
        T03 = mat1*mat2*mat3*mat4;
        std::cout << "T03_invert: " << std::endl;
        T03_invert = transposematrix(T03);
        T03_invert.output();

        std::vector<std::vector<double>> sol_theta_4_5_6_T03 = inv_vec_sol_theta4_5_6(T03_invert, _pos);
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

/* Calculate R36 and return final solution for Theta 4, 5, 6
 * @requires:   R03_invert  :           TMatrix 4x4 Rotationmatrix from joint 1 to 3
 *              _pos    :               Input values of the user containing position and orientation of the end position
 * @ensures:    solution_theta_4_5_6 :  Vector inside a vector that contains all solutions for theta 4,5,6 that are
 *                                      inside the boundaries
 * */
std::vector<std::vector<double>> InvKinematics::inv_vec_sol_theta4_5_6(TMatrix R03_invert, SixDPos* _pos){

    TMatrix R06(_pos->get_A(), _pos->get_B(), _pos->get_C(),0,0,0);

    TMatrix R36 = R03_invert * R06;
    std::array<double, 10> solution_standard_4_5_6;
    solution_standard_4_5_6 = inv_gettheta4_5_6(R36);
    std::vector<vector<double>> solution_theta_4_5_6 = inv_checklimits_theta4_5_6(solution_standard_4_5_6);

    return solution_theta_4_5_6;
}

/* Check if the Theta 4, 5 and 6 are inside the robots boundaries
 * @requires:   solution_standard_4_5_6  :  double array[10] that contains all solutions of theta 4, 5,6
 * @ensures:    solution_theta_4_5_6 :     Vector inside a vector that contains all solutions for theta 4,5,6 that are
 *                                          inside the boundaries
 * */
std::vector<std::vector<double>> InvKinematics::inv_checklimits_theta4_5_6(std::array<double, 10> solution_standard_4_5_6){

    double id = 0;
    std::vector<double> config1;
    config1.clear();
    std::vector<double> config2;
    config2.clear();
    std::vector<double> config3;
    config3.clear();
    std::vector<double> config4;
    config4.clear();
    std::vector<double> config5;
    config5.clear();
    std::vector<double> config6;
    config6.clear();
    std::vector<double> config7;
    config7.clear();
    std::vector<double> config8;
    config8.clear();
    std::vector<double> vec_id;
    std::vector<vector<double>> sol_theta_4_5_6;
    sol_theta_4_5_6.clear();
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
    if(theta5_l <= solution_standard_4_5_6[4] && solution_standard_4_5_6[4] <=  theta5_u){
        if(theta4_l <= solution_standard_4_5_6[0] && solution_standard_4_5_6[0] <= theta4_u){
            if(theta6_l <= solution_standard_4_5_6[6] && solution_standard_4_5_6[6] <= theta6_u){
                //Config 1
                id++;
                config1.push_back(solution_standard_4_5_6[0]);
                config1.push_back(solution_standard_4_5_6[4]);
                config1.push_back(solution_standard_4_5_6[6]);
                sol_theta_4_5_6.push_back(config1);
            }
            if(theta6_l <= solution_standard_4_5_6[7] && solution_standard_4_5_6[7] <= theta6_u){
                //Config 2
                id++;
                config2.push_back(solution_standard_4_5_6[0]);
                config2.push_back(solution_standard_4_5_6[4]);
                config2.push_back(solution_standard_4_5_6[7]);
                sol_theta_4_5_6.push_back(config2);
            }
        }
        if(theta4_l <= solution_standard_4_5_6[1] && solution_standard_4_5_6[1] <= theta4_u){
            if(theta6_l <= solution_standard_4_5_6[6] && solution_standard_4_5_6[6] <= theta6_u){
                //Config 3
                id++;
                config3.push_back(solution_standard_4_5_6[1]);
                config3.push_back(solution_standard_4_5_6[4]);
                config3.push_back(solution_standard_4_5_6[6]);
                sol_theta_4_5_6.push_back(config3);
            }
            if(theta6_l <= solution_standard_4_5_6[7] && solution_standard_4_5_6[7] <= theta6_u){
                //Config 4
                id++;
                config4.push_back(solution_standard_4_5_6[1]);
                config4.push_back(solution_standard_4_5_6[4]);
                config4.push_back(solution_standard_4_5_6[7]);
                sol_theta_4_5_6.push_back(config4);
            }
        }
    }

    if(theta5_l <= solution_standard_4_5_6[5] && solution_standard_4_5_6[5]<= theta5_u){
        if(theta4_l <= solution_standard_4_5_6[2] && solution_standard_4_5_6[2] <= theta4_u){
            if(theta6_l <= solution_standard_4_5_6[8] && solution_standard_4_5_6[8] <= theta6_u){
                //Config 5
                id++;
                config5.push_back(solution_standard_4_5_6[2]);
                config5.push_back(solution_standard_4_5_6[5]);
                config5.push_back(solution_standard_4_5_6[8]);
                sol_theta_4_5_6.push_back(config5);
            }
            if(theta6_l <= solution_standard_4_5_6[9] && solution_standard_4_5_6[9] <= theta6_u){
                //Config 6
                id++;
                config6.push_back(solution_standard_4_5_6[2]);
                config6.push_back(solution_standard_4_5_6[5]);
                config6.push_back(solution_standard_4_5_6[9]);
                sol_theta_4_5_6.push_back(config6);
            }
        }
        if(theta4_l <= solution_standard_4_5_6[3] && solution_standard_4_5_6[3] <= theta4_u){
            if(theta6_l <= solution_standard_4_5_6[8] && solution_standard_4_5_6[8]<= theta6_u){
                //Config 7
                id++;
                config7.push_back(solution_standard_4_5_6[3]);
                config7.push_back(solution_standard_4_5_6[5]);
                config7.push_back(solution_standard_4_5_6[8]);
                sol_theta_4_5_6.push_back(config7);
            }
            if(theta6_l <= solution_standard_4_5_6[9] && solution_standard_4_5_6[9] <= theta6_u){
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

/* Calculate solutions for the first other case
 * @requires:   Theta1  :   Rotation of the first joint in degree
 *              d1      :   distance between the rootjoint and the wcp in mm
 *              wcp     :   double array[3] that contains position of the robots wrist center point (x,y,z)
 *              _pos    :   Input values of the user containing position and orientation of the end position
 * @ensures:    sol_othercase_1_config_vec :    a configuration vector with all possible configurations of the joints for
 *                                              the first othercase
 * */
std::vector<Configuration*>* InvKinematics::inv_othercase_1(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos){
    double id = 0;
    std::cout << "Other case 1: " << std::endl;
    std::vector<Configuration*>* sol_othercase_1_vec_config;
    double dpx = d1 + m;
    double dpy = wcp[2] -n;
    double theta1_1 = theta1 + 180;
    theta2_theta3 = inv_backwardcase(dpx, dpy);
    sol_othercase_1_vec = inv_checklimits_theta1_2_3(theta1_1, theta2_theta3);
    id = sol_othercase_1_vec.at(0).at(0);
    if(id == 4){
        std::cout << "No possible configuration for the robot" << endl;
        return NULL;
    }
    else {
        if (id == 2 || id == 3) {
            sol_othercase_1_vec.at(0).at(0) = 200 + id;
        } else if (id == 11) {
            sol_othercase_1_vec.at(0).at(0) = 200 + id;
            sol_othercase_1_vec.at(1).at(0) = 200 + 12;
        }

        sol_othercase_1_vec_config = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_othercase_1_vec);
        // size1_1 is the number of configurations inside the vector
        double size1_1 = sol_othercase_1_vec_config->size();
        for (int i = 0; i < size1_1; i++) {
            // push every configuration into the complete vector
            solution_othercase_1_vec->push_back(sol_othercase_1_vec_config->at(i));
        }
        if (size1_1 == 0) {
            std::cout << "No configuration added to solution_vec for other case 1 ("
                      << sol_othercase_1_vec_config->size() << ")" << std::endl;
        } else {
            std::cout << "Configuration added to solution_vec for other case 1 ("
                      << sol_othercase_1_vec_config->size() << ")" << std::endl;
        }

    }
    return sol_othercase_1_vec_config;
}

/* Calculate solutions for the second other case
 * @requires:   Theta1  :   Rotation of the first joint in degree
 *              d1      :   distance between the rootjoint and the wcp in mm
 *              wcp     :   double array[3] that contains position of the robots wrist center point (x,y,z)
 *              _pos    :   Input values of the user containing position and orientation of the end position
 * @ensures:    sol_othercase_1_config_vec :    a configuration vector with all possible configurations of the joints for
 *                                              the first othercase
 * */
std::vector<Configuration*>* InvKinematics::inv_othercase_2(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos){
    double id = 0;
    std::cout << "Other case 2: " << std::endl;
    std::vector<Configuration*>* sol_othercase_2_vec_config;
    double dpx = d1 + m;
    double dpy = wcp[2] -n;
    double theta1_1 = theta1 - 180;
    theta2_theta3 = inv_backwardcase(dpx, dpy);
    sol_othercase_2_vec = inv_checklimits_theta1_2_3(theta1_1, theta2_theta3);
    id = sol_othercase_2_vec.at(0).at(0);
    if(id == 4){
        std::cout << "No possible configuration for the robot" << endl;
        return NULL;
    }
    else {
        if (id == 2 || id == 3) {
            sol_othercase_2_vec.at(0).at(0) = 200 + id;
        } else if (id == 11) {
            sol_othercase_2_vec.at(0).at(0) = 200 + id;
            sol_othercase_2_vec.at(1).at(0) = 200 + 12;
        }

        sol_othercase_2_vec_config = inv_add_case_to_vec(theta1_1, d1, wcp, _pos, sol_othercase_2_vec);
        // size1_1 is the number of configurations inside the vector
        double size1_1 = sol_othercase_2_vec_config->size();
        for (int i = 0; i < size1_1; i++) {
            // push every configuration into the complete vector
            solution_othercase_2_vec->push_back(sol_othercase_2_vec_config->at(i));
        }
        if (size1_1 == 0) {
            std::cout << "No configuration added to solution_vec for other case 2 ("
                      << sol_othercase_2_vec_config->size() << ")" << std::endl;
        } else {
            std::cout << "Configuration added to solution_vec for other case 2 ("
                      << sol_othercase_2_vec_config->size() << ")" << std::endl;
        }
    }
    return sol_othercase_2_vec_config;
}



vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    //you should call your inverse kinematics functions here!

    double theta1;
    double id = 0;
    std::array<double, 4> dTCP = {0,0,-215,1};                                                                        //Vector of the distance from TCP to WCP

    std::cout << "-----------Inverse Kinematics------------------------------" << std::endl;
    std::cout << "X of sixDPos " <<_pos->get_X() << std::endl;
    std::cout << "Y of sixDPos " <<_pos->get_Y() << std::endl;
    std::cout << "Z of sixDPos " <<_pos->get_Z() << std::endl;
    std::cout << "A of sixDPos " <<_pos->get_A() << std::endl;
    std::cout << "B of sixDPos " <<_pos->get_B() << std::endl;
    std::cout << "C of sixDPos " <<_pos->get_C() << std::endl;

    //Position and Orientation ot the Tool Center Point (TCP)
    TMatrix TCP(_pos->get_A(),_pos->get_B(),_pos->get_C(),
                _pos->get_X()*1000,_pos->get_Y()*1000,_pos->get_Z()*1000);                                                            //Transformation Matrix for the TCP inside of the global coordinate system
                                                                                                                                                    //Calculation of wrist center point
    std::array<double, 3> wcp;

    //Calculation of the Wrist Center Point (WCP)
    wcp[0] = _pos->get_X() * 1000 - (215 * TCP.get_element(0, 2));
    wcp[1] = _pos->get_Y() * 1000 - (215 * TCP.get_element(1, 2));
    wcp[2] = _pos->get_Z() * 1000 - (215 * TCP.get_element(2, 2));



    for (int i = 0; i < 3; ++i) {
        std::cout << "wcp :" << wcp[i] << std::endl;
    }

    //Checking for elbow singularity
    double X = sqrt(pow(wcp[0],2) + pow(wcp[1],2)) - m;
    double Y = wcp[2] - n;

    double X_square_Y_square = pow(X,2) + pow(Y,2);
    double d2 = sqrt(o * o + b * b);

    if(X_square_Y_square >= (pow((a + d2), 2)-margin_point)){
        wcp[0] = sqrt(pow(X+m, 2) - pow(wcp[1],2));
        wcp[2] = Y + n;
        std::cout << "There is an elbow singularity. wcp[0] " << wcp[0] << " ; wcp[2] " << wcp[2]  << std::endl;
    }

    //Case 1
    if(wcp[0]>0 && wcp[1]>=0)
    {
        cout << "Fall 1" << endl;
        theta1= ((-1) * atan(wcp[1] / wcp[0])) * 180 / M_PI;
        cout << "theta1: " << theta1 << endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if(-185 < theta1 && theta1 < -175 || 185 > theta1 && theta1 > 175 || -5 < theta1 && theta1 < 5){
            solution_vec = inv_checktheta(theta1, d1, wcp, _pos);
        }
        else {
            //solution_vec contains the possible configurations for a standardcase for all joints
            solution_standard = inv_standardcase(theta1, d1, wcp);
            solution_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, solution_standard);

            //othercases
            sol_othercase_1_vec_config = inv_othercase_1(theta1, d1, wcp, _pos);
            sol_othercase_2_vec_config = inv_othercase_2(theta1, d1, wcp, _pos);

            // add configs from othercases to solution
            if(sol_othercase_1_vec_config != NULL){
                double size_othercase1 = sol_othercase_1_vec_config->size();
                for(int i = 0; i < size_othercase1; i++){
                    //add configurations to solution vector
                    solution_vec ->push_back(sol_othercase_1_vec_config->at(i));
                }
            }
            if(sol_othercase_2_vec_config != NULL){
                double size_othercase2 = sol_othercase_2_vec_config->size();
                for(int i = 0; i < size_othercase2; i++){
                    //add configurations to solution vector
                    solution_vec ->push_back(sol_othercase_2_vec_config->at(i));
                }
            }
        }
    }

    //Case 2
    else if(wcp[0]<0 && wcp[1]<=0)
    {
        cout << "Fall 2" << endl;
        theta1= 180 - atan(wcp[1] / wcp[0]) * 180 / M_PI;
        cout << "theta1: " << theta1 << endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < theta1 && theta1 < -175) || (185 > theta1 && theta1 > 175) || (-5 < theta1 && theta1 < 5)){
            solution_vec = inv_checktheta(theta1, d1, wcp, _pos);
        }
        else {
            //solution_vec contains the possible configurations for a standardcase for all joints
            solution_standard = inv_standardcase(theta1, d1, wcp);
            solution_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, solution_standard);

            //othercases
            sol_othercase_1_vec_config = inv_othercase_1(theta1, d1, wcp, _pos);
            sol_othercase_2_vec_config = inv_othercase_2(theta1, d1, wcp, _pos);

            // add configs from othercases to solution
            if (sol_othercase_1_vec_config != NULL) {
                double size_othercase1 = sol_othercase_1_vec_config->size();
                for (int i = 0; i < size_othercase1; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_1_vec_config->at(i));
                }
            }
            if (sol_othercase_2_vec_config != NULL) {
                double size_othercase2 = sol_othercase_2_vec_config->size();
                for (int i = 0; i < size_othercase2; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_2_vec_config->at(i));
                }
            }
        }
    }

    //Case 3
    else if(wcp[0]>0 && wcp[1]<=0)
    {
        cout << "Fall 3" << endl;
        theta1= (-1) * atan(wcp[1] / wcp[0]) * 180 / M_PI;
        std::cout << "theta1: " << theta1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < theta1 && theta1 < -175) || (185 > theta1 && theta1 > 175) || (-5 < theta1 && theta1 < 5)){
            solution_vec = inv_checktheta(theta1, d1, wcp, _pos);
        }
        else {
            //solution_vec contains the possible configurations for a standardcase for all joints
            solution_standard = inv_standardcase(theta1, d1, wcp);
            solution_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, solution_standard);


            //othercases
            sol_othercase_1_vec_config = inv_othercase_1(theta1, d1, wcp, _pos);
            sol_othercase_2_vec_config = inv_othercase_2(theta1, d1, wcp, _pos);

            // add configs from othercases to solution
            if (sol_othercase_1_vec_config != NULL) {
                double size_othercase1 = sol_othercase_1_vec_config->size();
                for (int i = 0; i < size_othercase1; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_1_vec_config->at(i));
                }
            }
            if (sol_othercase_2_vec_config != NULL) {
                double size_othercase2 = sol_othercase_2_vec_config->size();
                for (int i = 0; i < size_othercase2; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_2_vec_config->at(i));
                }
            }
        }
    }

    //Case 4
    else if(wcp[0]<0 && wcp[1]>=0)
    {
        cout << "Fall 4" << endl;
        theta1= (-1)*(180 - atan(wcp[1] / (-1 * wcp[0])) * 180 / M_PI);
        std::cout << "theta1: " << theta1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < theta1 && theta1 < -175) || (185 > theta1 && theta1 > 175) || (-5 < theta1 && theta1 < 5)){
            solution_vec = inv_checktheta(theta1, d1, wcp, _pos);
        }
        else {
            //solution_vec contains the possible configurations for a standardcase for all joints
            solution_standard = inv_standardcase(theta1, d1, wcp);
            solution_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, solution_standard);


            //othercases
            sol_othercase_1_vec_config = inv_othercase_1(theta1, d1, wcp, _pos);
            sol_othercase_2_vec_config = inv_othercase_2(theta1, d1, wcp, _pos);

            // add configs from othercases to solution
            if (sol_othercase_1_vec_config != NULL) {
                double size_othercase1 = sol_othercase_1_vec_config->size();
                for (int i = 0; i < size_othercase1; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_1_vec_config->at(i));
                }
            }
            if (sol_othercase_2_vec_config != NULL) {
                double size_othercase2 = sol_othercase_2_vec_config->size();
                for (int i = 0; i < size_othercase2; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_2_vec_config->at(i));
                }
            }
        }
    }

    //Special cases
    //Special case 1:
    else if((wcp[0] <= 0.001 && wcp[0] >= -0.001) && wcp[1] > 0){
        std::cout << "Special Case 1: " << std::endl;
        if(wcp[1] > m){
            double d1 = wcp[1];
            theta1 = -90;
            double dpx = wcp[1] - m;
            double dpy = wcp[2] - n;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
                sol_theta1_special1_1.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }

            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
               // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 1 (y > m, theta = -90) (" << sol_specialcase1_1_vec->size() << ") for y > m" << std:: endl;
            }
            //new theta
            theta1 = 90;
            dpx = wcp[1]+m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_2);
            // size1_2 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 1 (y > m, theta = 90) (" << sol_specialcase1_2_vec->size() << ")" << std:: endl;
            }
            // check if all configurations got added to solution_vec
            if(solution_vec->size() == size1_1+size1_2){
                std::cout << "All configurations added to solution_vec (" << solution_vec->size() << ") for y > m" << std:: endl;
            }

        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            theta1 = -90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
                sol_theta1_special1_1.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 1 (y < m, theta = -90) (" << sol_specialcase1_1_vec->size() << ")" << std:: endl;
            }
            //new theta
            theta1 = 90;
            dpx = wcp[1] + m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_2);
            // size1_2 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 1 (y < m, theta = 90) (" << sol_specialcase1_2_vec->size() << ") for y < m" << std:: endl;
            }
            // check if all configurations got added to solution_vec
            if(solution_vec->size() == size1_1+size1_2){
                std::cout << "All configurations added to solution_vec (" << solution_vec->size() << ") for y < m" << std:: endl;
            }
        }
    }
    //special case 2:
    else if(wcp[0]==0 && wcp[1]<0){
        std::cout << "Special Case 2: " << std::endl;

        if(wcp[1] > m){
            double d1 = wcp[1];
            theta1 = 90;
            double dpx = wcp[1] - m;
            double dpy = wcp[2] - n;
            theta2_theta3 = inv_forwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 100+id;
                sol_theta1_special1_1.at(1).at(0) = 100+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 2 (y > m, theta = 90) (" << sol_specialcase1_1_vec->size() << ") for y > m" << std:: endl;
            }
            //new theta
            theta1 = -90;
            dpx = wcp[1]+m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_2);
            // size1_2 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 2 (y > m, theta = -90) (" << sol_specialcase1_2_vec->size() << ")" << std:: endl;
            }
            // check if all configurations got added to solution_vec
            if(solution_vec->size() == size1_1+size1_2){
                std::cout << "All configurations added to solution_vec (" << solution_vec->size() << ") for y > m" << std:: endl;
            }
        }

        if(wcp[1] < m){
            double d1 = wcp[1];
            theta1 = 90;
            double dpx = m -  wcp[1];
            double dpy = wcp[2] - n;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_1 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_1.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_1.at(0).at(0) = 200+id;
                sol_theta1_special1_1.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }

            sol_specialcase1_1_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_1);
            // size1_1 is the number of configurations inside the vector
            double size1_1 = sol_specialcase1_1_vec->size();
            for(int i = 0; i < size1_1; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_1_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 2 (y < m, theta = 90) (" << sol_specialcase1_1_vec->size() << ")" << std:: endl;
            }
            //new theta
            theta1 = -90;
            dpx = wcp[1] + m;
            theta2_theta3 = inv_backwardcase(dpx, dpy);
            sol_specialcase1_2 = inv_checklimits_theta1_2_3(theta1, theta2_theta3);
            id = sol_specialcase1_2.at(0).at(0);
            if(id == 2 || id == 3){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
            }

            else if(id == 11){
                sol_theta1_special1_2.at(0).at(0) = 200+id;
                sol_theta1_special1_2.at(1).at(0) = 200+12;
            }

            else if(id == 4){
                std::cout << "No possible configuration for the robot" << endl;
            }
            sol_specialcase1_2_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, sol_specialcase1_2);
            // size1_2 is the number of configurations inside the vector
            double size1_2 = sol_specialcase1_2_vec->size();
            for(int i = 0; i < size1_2; i++){
                // push every configuration into the complete vector
                solution_vec->push_back(sol_specialcase1_2_vec->at(i));
            }
            if(size1_1 == 0){
                std::cout << "No configuration added to solution_vec for special case 2 (y < m, theta = -90) (" << sol_specialcase1_2_vec->size() << ") for y < m" << std:: endl;
            }
            // check if all configurations got added to solution_vec
            if(solution_vec->size() == size1_1+size1_2){
                std::cout << "All configurations added to solution_vec (" << solution_vec->size() << ") for y < m" << std:: endl;
            }
        }
    }

    // checking for shoulder singularity
    if((wcp[0] + margin_point <= 0 && wcp[0] + margin_point >= 0) && (wcp[1] + margin_point <= 0 && wcp[1] + margin_point >= 0)){
        std::cout << "There is a shoulder singularity." << std::endl;
        //theta 1 is chosen as 0
        theta1 = 0;
        std::cout << "theta1: " << theta1 << std::endl;
        double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
        std::cout << "d1: " << d1 << std::endl;

        if((-185 < theta1 && theta1 < -175) || (185 > theta1 && theta1 > 175) || (-5 < theta1 && theta1 < 5)){
            solution_vec = inv_checktheta(theta1, d1, wcp, _pos);
        }
        else {
            //solution_vec contains the possible configurations for a standardcase for all joints
            solution_standard = inv_standardcase(theta1, d1, wcp);
            solution_vec = inv_add_case_to_vec(theta1, d1, wcp, _pos, solution_standard);


            //othercases
            sol_othercase_1_vec_config = inv_othercase_1(theta1, d1, wcp, _pos);
            sol_othercase_2_vec_config = inv_othercase_2(theta1, d1, wcp, _pos);

            // add configs from othercases to solution
            if (sol_othercase_1_vec_config != NULL) {
                double size_othercase1 = sol_othercase_1_vec_config->size();
                for (int i = 0; i < size_othercase1; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_1_vec_config->at(i));
                }
            }
            if (sol_othercase_2_vec_config != NULL) {
                double size_othercase2 = sol_othercase_2_vec_config->size();
                for (int i = 0; i < size_othercase2; i++) {
                    //add configurations to solution vector
                    solution_vec->push_back(sol_othercase_2_vec_config->at(i));
                }
            }
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

    //returning solution_vec
    if(solution_vec != nullptr ) {//return solutions;
        std::cout << "There are " << solution_vec->size() << " possible configurations." << std::endl;
        return solution_vec;
    }

    //returning null pointer
    else{
        Configuration *no_config;
        std::cout << "There are no possible configurations for this position." << std::endl;
        return nullptr;
    }
}