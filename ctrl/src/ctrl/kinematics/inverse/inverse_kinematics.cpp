#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;
#define PI 3.14159265
#define d_6 1.435  //Here the distance is 1435mm
#define m 0.330   // the length of the m is 330mm
#define n 0.645   // the length of the n is 645mm
#define o 0.115   // the length of the o is 115mm
#define a 1.150   // the length of the a is 1150mm
#define b 1.220   // the length of the b is 1220mm

std::vector<double> find_theta1(double X_c, double Y_c);
void find_theta2_theta3(double X_c, double Y_c, double Z_c, vector<double>& arr2, vector<double>& arr3);

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->push_back(new Configuration({ 0,0,1,0,0,0 }));
    double new_X = (*_pos)[0];
    double new_Y = (*_pos)[1];
    double new_Z = (*_pos)[2];
    double new_roll = (*_pos)[3];
    double new_pitch = (*_pos)[4];
    double new_yaw = (*_pos)[5];


    // Les's find matrix R0_6 by using Raw Yaw and Pitch angle 
    double R0_6[3][3];
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;



    r11 = cos(new_roll) * cos(new_pitch);
    r12 = -(sin(new_roll) * cos(new_yaw)) + (cos(new_roll) * sin(new_pitch) * sin(new_yaw));
    r13 = (sin(new_roll) * sin(new_yaw)) + (cos(new_roll) * sin(new_pitch) * cos(new_yaw));
    r21 = sin(new_roll) * cos(new_pitch);
    r22 = (cos(new_roll) * cos(new_yaw)) + (sin(new_roll) * sin(new_pitch) * sin(new_yaw));
    r23 = -(cos(new_roll) * sin(new_yaw)) + (sin(new_roll) * sin(new_pitch) * cos(new_yaw));
    r31 = -sin(new_pitch);
    r32 = cos(new_pitch) * sin(new_yaw);
    r33 = cos(new_pitch) * cos(new_yaw);

    R0_6[0][0] = r11;
    R0_6[0][1] = r12;
    R0_6[0][2] = r13;
    R0_6[1][0] = r21;
    R0_6[1][1] = r22;
    R0_6[1][2] = r23;
    R0_6[2][0] = r31;
    R0_6[2][1] = r32;
    R0_6[2][2] = r33;

    //let's compute the wrist center position Oc [Xc, Yc, Zc]
    double X_c, Y_c, Z_c;
    X_c = new_X - (d_6 * r13);
    Y_c = new_Y - (d_6 * r23);
    Z_c = new_Z - (d_6 * r33);


    //Compute the theta1 here
    std::vector<double> theta1_arr, theta2_arr, theta3_arr;
    theta1_arr = find_theta1(X_c, Y_c);

    //vector<double>theta1_arr;
    //find_theta1(X_c, Y_c, theta1_arr);

    //Compute the theta2 and theta3 here
    for (auto elem1 : theta1_arr)
    {
        if (-185 < elem1 < 185)
        {
            find_theta2_theta3(X_c, Y_c, Z_c, theta2_arr, theta3_arr);
            for (int i = 0; i < theta2_arr.size(); i++)
            {
                if (-140 < theta2_arr[i] < -5 && -120 < theta3_arr[i] < 168)
                {
                    solutions->push_back(new Configuration({ elem1,theta2_arr[i],theta3_arr[i],0,0,0 }));
                }
                else
                    break;
            }
        }
        else if (elem1 < -185 || elem1 >185)
            break;

    }




    /*
      solutions->push_back(new Configuration({0,0,1,0,0,0}));
      solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
      solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));
  */
    return solutions;
}

/*As we know from the Group one presentation if theta1 is between -175 and -175 or
    from 175 to 185 then we have two forward solutions and only one backward solution.
    In addition, when we have angle from -5 to 5, then we have one forward soluion
    and two backward solution
    */

std::vector<double> find_theta1(double X_c, double Y_c)
{

    //computing the taninverse using general mathmatics formula
    double theta1;
    theta1 = atan2(Y_c, X_c) * 180 / PI;
    std::vector<double> arr;

    //cheaking the limit of theta1 and pushing the theta1 value for forward and backward cases

    if (-175 < theta1 < 175) {

        if (X_c > 0 && Y_c < 0) {

            arr.push_back(-theta1);
            arr.push_back(-(180 + theta1));
        }
        else if (X_c < 0 && Y_c < 0) {

            arr.push_back(-theta1);
            arr.push_back(-(180 + theta1));
        }
        else if (X_c < 0 && Y_c > 0) {

            arr.push_back(-theta1);
            arr.push_back(180 - theta1);
        }
        else if (X_c > 0 && Y_c > 0) {
            arr.push_back(-theta1);
            arr.push_back(180 - theta1);
        }
        else if (X_c == 0 && Y_c > 0) {
            arr.push_back(-theta1);
            arr.push_back(180 - theta1);

        }
        else if (X_c == 0 && Y_c < 0) {
            arr.push_back(-theta1);
            arr.push_back(-(180 + theta1));

        }
    }


    if (-185 < (-theta1) < -175) {
        double case1 = -theta1;
        double case2 = -(180 - theta1);
        double case3 = (180 - theta1);
        arr.push_back(case1);
        arr.push_back(case2);
        arr.push_back(case3);
    }
    else if (175 < (-theta1) < 185) {
        double case1 = -theta1;
        double case2 = -(180 - theta1);
        double case3 = (180 - theta1);
        arr.push_back(case1);
        arr.push_back(case2);
        arr.push_back(case3);
    }

    else if (-5 < (-theta1) < 5) {
        double case1 = -theta1;
        double case2 = -(180 - theta1);
        double case3 = (180 - theta1);
        arr.push_back(case1);
        arr.push_back(case2);
        arr.push_back(case3);
    }

    return arr;
}


void find_theta2_theta3(double X_c, double Y_c, double Z_c, vector<double>& arr2, vector<double>& arr3)
{
    double d_1, d_2, d_3, Px_dash, Py_dash, theta2, theta3, beta1, alpha1, alpha2;
    d_1 = sqrt(X_c * X_c + Y_c * Y_c);
    d_2 = sqrt(b * b + o * o);
    if (d_1 > m)
    {
        Px_dash = d_1 - m;
        Py_dash = Z_c - n;
        d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
        beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
        alpha1 = asin(sin(beta1) * (d_2 / d_3)) * 180 / PI;
        alpha2 = asin(Py_dash / d_3) * 180 / PI;
        //forward elbow down
        theta2 = -(alpha2 - alpha1);
        theta3 = beta1 - (asin(b / d_2) * 180 / PI) - 90;
        arr2.push_back(theta2);
        arr3.push_back(theta3);

        // forward elbow up
        theta2 = -(alpha1 + alpha2);
        theta3 = 360 - beta1 - (asin(b / d_2) * 180 / PI) - 90;
        arr2.push_back(theta2);
        arr3.push_back(theta3);


    }
    else if (d_1 < m)
    {
        Px_dash = m - d_1;
        Py_dash = Z_c - n;
        d_3 = sqrt(Px_dash * Px_dash + Py_dash * Py_dash);
        beta1 = acos(((d_3 * d_3) - (a * a) - (d_2 * d_2)) / (-2 * a * d_2)) * 180 / PI;
        alpha1 = asin(sin(beta1) * (d_2 / d_3)) * 180 / PI;
        alpha2 = asin(Py_dash / d_3) * 180 / PI;
        //bacward elbow down
        theta2 = (alpha2 - alpha1) - 180;
        theta3 = -(90 - (beta1 - (asin(b / d_2) * 180 / PI)));
        arr2.push_back(theta2);
        arr3.push_back(theta3);

        //backward elbow down
        theta2 = (alpha1 + alpha2) - 180;
        theta3 = 270 - beta1 - (asin(b / d_2) * 180 / PI);
        arr2.push_back(theta2);
        arr3.push_back(theta3);

    }
}