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
std::vector<vector<double>> find_rotational_matrix_R0_3(double theta_1, double theta_2, double theta_3);
void find_theta4_theta5_theta6(double a_x, double a_y, double a_z, double n_z, double s_z, vector<double>& arr4, vector<double>& arr5, vector<double>& arr6);


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->push_back(new Configuration({ 0,0,1,0,0,0 }));

    double new_X =  (*_pos) [0];
    double new_Y = (*_pos) [1];
    double new_Z = (*_pos) [2];
    double new_roll = (*_pos) [3];
    double new_pitch = (*_pos) [4];
    double new_yaw = (*_pos) [5];



    // Les's find matrix R0_6 by using Raw Yaw and Pitch angle 
    //double R0_6[3][3];
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

    std::vector<vector<double>> R0_6
    {
        {r11,r12,r13},
        {r21,r22,r23},
        {r31,r32,r33}
    };

    //let's compute the wrist center position Oc [Xc, Yc, Zc]
    double X_c, Y_c, Z_c;
    X_c = new_X - (d_6 * r13);
    Y_c = new_Y - (d_6 * r23);
    Z_c = new_Z - (d_6 * r33);

   
    //Compute the theta1 here
    std::vector<double> theta1_arr, theta2_arr, theta3_arr, theta4_arr, theta5_arr, theta6_arr;
    theta1_arr = find_theta1(X_c, Y_c);

    //vector<double>theta1_arr;
    //find_theta1(X_c, Y_c, theta1_arr);

    //Compute the theta2 and theta3 here

    std::vector<vector<double>> inv_R0_3, R3_6;
    R3_6 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };
    double a_x, a_y, a_z, n_z, s_z;
    double th_1, th_2, th_3;

    /*
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
    */

    std::vector<int>::size_type sz = theta1_arr.size();

    for (unsigned j = 0; j < sz; j++)
    {
        th_1 = theta1_arr[j];

        if (-185 < theta1_arr[j] < 185)
        {
            find_theta2_theta3(X_c, Y_c, Z_c, theta2_arr, theta3_arr);

            std::vector<int>::size_type az = theta2_arr.size();
            for (unsigned i = 0; i < az; i++)
            {


                th_2 = theta2_arr[i];
                th_3 = theta3_arr[i];
                cout << "Theta 1: " << th_1 << " Theta 2: " << th_2 << " Theta 3: " << th_3 << endl;


                // Here we are finding base rotational matrix R0_3
                inv_R0_3 = find_rotational_matrix_R0_3(th_1, th_2, th_3);



                // Here we are multiplying first two rotational matrix R0_3 and R0_6
                for (int l = 0; l < 3; l++)
                {
                    for (int j = 0; j < 3; j++)
                    {

                        for (int k = 0; k < 3; k++)
                        {
                            R3_6[l][j] += inv_R0_3[l][k] * R0_6[k][j];
                        }
                    }
                }


                //Here are finding the angles theta 4, theta 5, theta 6
                a_x = R3_6[0][2];
                a_y = R3_6[1][2];
                a_z = R3_6[2][2];
                n_z = R3_6[2][0];
                s_z = R3_6[2][1];
                find_theta4_theta5_theta6(a_x, a_y, a_z, n_z, s_z, theta4_arr, theta5_arr, theta6_arr);

                if (-140 < theta2_arr[i] < -5 && -120 < theta3_arr[i] < 168)
                {
                    solutions->push_back(new Configuration({ theta1_arr[j],theta2_arr[i],theta3_arr[i],theta4_arr[i],theta5_arr[i],theta6_arr[i] }));
                }
                else
                    break;
            }
        }
        else if (theta1_arr[j] < -185 || theta1_arr[j] >185)
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
        alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
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
        alpha1 = asin(sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI;
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

std::vector<vector<double>> find_rotational_matrix_R0_3(double theta_1, double theta_2, double theta_3)
{
    double nx, ny, nz, sx, sy, sz, ax, ay, az, th, al;
    double alpha[4] = { 180, 90, 0, 90 };
    double theta[4] = { 0, theta_1, theta_2, theta_3 - 90 };
    std::vector<vector<double>> R0_1, R1_2, R2_3, R3_4, R0_2, R2_4, R0_3, new_R0_3;

    R0_2 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    R2_4 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    R0_3 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    new_R0_3 =
    {
            {0,0,0},
            {0,0,0},
            {0,0,0}
    };

    //Here we are finding the rotation matrix R0_1
    {
        th = theta[0];
        al = alpha[0];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * cos(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R0_1 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sy,ay}
        };
    }

    //Here we are finding the rotation matrix R1_2
    {
        th = theta[1];
        al = alpha[1];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * cos(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R1_2 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sy,ay}
        };
    }

    //Here we are finding the rotation matrix R2_3
    {
        th = theta[2];
        al = alpha[2];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * cos(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);;
        R2_3 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sy,ay}
        };
    }

    //Here we are finding the rotation matrix R3_4
    {
        th = theta[3];
        al = alpha[3];
        nx = cos(th * PI / 180);
        sx = -sin(th * PI / 180) * cos(al * PI / 180);
        ax = sin(th * PI / 180) * cos(al * PI / 180);
        ny = sin(th * PI / 180);
        sy = cos(th * PI / 180) * cos(al * PI / 180);
        ay = -cos(th * PI / 180) * sin(al * PI / 180);
        nz = 0;
        sz = sin(al * PI / 180);
        az = cos(al * PI / 180);
        R3_4 =
        {
            {nx,sx,ax},
            {ny,sy,ay},
            {nz,sy,ay}
        };
    }

    // Here we are multiplying first two rotational matrix R0_1 and R1_2
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R0_2[i][j] += R0_1[i][k] * R1_2[k][j];
            }
        }
    }

    // Here we are multiplying last two rotational matrix R2_3 and R3_4
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R2_4[i][j] += R2_3[i][k] * R3_4[k][j];
            }
        }
    }

    // Here we are multiplying the result of multiplication and we will get the R0_3
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {

            for (int k = 0; k < 3; k++)
            {
                R0_3[i][j] += R0_2[i][k] * R2_4[k][j];
            }
        }
    }

    // finidng the inverse matrix of R0_3
    double determinant = 0;

    //here we are solving the determinant of the rotational matrix
    for (int i = 0; i < 3; i++)
    {
        determinant += R0_3[0][i] * ((R0_3[1][(i + 1) % 3] * R0_3[2][(i + 2) % 3]) - (R0_3[1][(i + 2) % 3] * R0_3[2][(i + 1) % 3]));
    }

    //Here we are going to find the inverse matrix using basic mathmatics formula
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            new_R0_3[i][j] = ((R0_3[(j + 1) % 3][(i + 1) % 3] * R0_3[(j + 2) % 3][(i + 2) % 3]) - (R0_3[(j + 1) % 3][(i + 2) % 3] * R0_3[(j + 2) % 3][(i + 1) % 3])) / determinant;
        }
    }


    return new_R0_3;

}

void find_theta4_theta5_theta6(double a_x, double a_y, double a_z, double n_z, double s_z, vector<double>& arr4, vector<double>& arr5, vector<double>& arr6)
{
    double theta_4, theta_5, theta_6;

    //here we are conmputing theta4
    theta_4 = atan2(-a_y, -a_x) * 180 / PI;
    if ((theta_4 - 360) > -350)
    {
        arr4.push_back(theta_4 - 360);
        arr4.push_back(theta_4);
    }
    else if ((theta_4 + 360) < 350)
    {
        arr4.push_back(theta_4 + 360);
        arr4.push_back(theta_4);
    }

    // here we are computing theta5 and theta6
    theta_5 = atan2(sqrt(1 - (a_z * a_z)), -a_z) * 180 / PI;
    theta_6 = atan2(s_z, n_z) * 180 / PI;
    if ((theta_6 - 360) > -350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6 - 360);
        arr6.push_back(theta_6);

    }
    else if ((theta_6 + 360) < 350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6 + 360);
        arr6.push_back(theta_6);
    }

    theta_5 = -theta_5;
    if (theta_4 > 0)
    {
        theta_4 = theta_4 - 180;
    }
    else
    {
        theta_4 = theta_4 + 180;
    }

    if ((theta_4 - 360) > -350)
    {
        arr4.push_back(theta_4 - 360);
        arr4.push_back(theta_4);
    }
    else if ((theta_4 + 360) < 350)
    {
        arr4.push_back(theta_4 + 360);
        arr4.push_back(theta_4);
    }

    if (theta_6 > 0)
    {
        theta_6 = theta_6 - 180;
    }
    else
    {
        theta_6 = theta_6 + 180;
    }

    if ((theta_6 - 360) > -350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6 - 360);
        arr6.push_back(theta_6);

    }
    else if ((theta_6 + 360) < 350)
    {
        arr5.push_back(theta_5);
        arr5.push_back(theta_5);
        arr6.push_back(theta_6 + 360);
        arr6.push_back(theta_6);
    }

}

