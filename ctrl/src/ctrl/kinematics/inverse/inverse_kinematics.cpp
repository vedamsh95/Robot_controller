#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!

    double o = 115;
    double m = 330;
    double n = 645;
    double a = 1150;
    double b = 1220;
    double phi1, phi2, phi3;

    std::array<double, 6> solution;
    std::array<double, 4> dTCP = {0,0,-215,1};                                                                        //Vector of the distance from TCP to WCP


    std::cout << "X of sixDPos" <<_pos->get_X() << std::endl;
    std::cout << "Y of sixDPos" <<_pos->get_Y() << std::endl;
    std::cout << "Z of sixDPos" <<_pos->get_Z() << std::endl;
    std::cout << "A of sixDPos" <<_pos->get_A() << std::endl;
    std::cout << "B of sixDPos" <<_pos->get_B() << std::endl;
    std::cout << "C of sixDPos" <<_pos->get_C() << std::endl;


    TMatrix TCP(_pos->get_A(),_pos->get_B(),_pos->get_C(),_pos->get_X(),_pos->get_Y(),_pos->get_Z());                                                            //Transformation Matrix for the TCP inside of the global coordinate system
    std::array<double, 4> wcp = TCP*dTCP;                                                                                                                                                 //Calculation of wrist center point


    for (int i = 0; i < 4; ++i) {
        std::cout << "wcp :" << wcp[i] << std::endl;
    }


    double d1= sqrt(wcp.at(0)*wcp.at(0)+wcp.at(1)*wcp.at(1));
    std::cout << "d1: " << d1 << std::endl;
    if(wcp[0]>0 && wcp[1]>0)
    {
        phi1=(-atan(wcp[1]/wcp[0])) * 180/M_PI;                                                                                          //calculated phi1
        std::cout << "phi1: " << phi1 << std::endl;
        if(d1>m && wcp[2]>n && -175<phi1<175)                                                                               //calculating all the forward cases
        {
            std::cout << "Fall1!! " << std::endl;
            double dpx = d1 - m;                                                                                            //calculating the x and y components of the
            double dpy = wcp[2] - n;                                                                                        //distance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                                                                        //direct distance between joint and wcp
            std::cout << "d3: " << d3 << std::endl;
            double d2 = sqrt(o * o + b * b);
            std::cout << "d2: " << d2 << std::endl;
            double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)))*180/M_PI;
            std::cout << "beta: " <<  beta << std::endl;
            double alpha1 = (asin(sin(beta)*180/M_PI * (d2 / d3)))*180/M_PI;
            std::cout << "alpha1: " << alpha1 << std::endl;
            double alpha2 = (asin(dpy / d3))*180/M_PI;
            std::cout << "alpha2: " << alpha2 << std::endl;

            double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
            double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down

            double phi3_f_u = 360 - beta - asin(b / d2)*180/M_PI - 90;                                                            //for forward elbow up
            double phi3_f_d = beta - asin(b / d2)*180/M_PI - 90;                                                                  //for forward elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_f_u < -5) {
                    if (-120 < phi3_f_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_f_u;
                        solution[2] = phi3_f_u;
                    }

                    if (-140 < phi2_f_d < -5) {
                        if (-120 < phi3_f_d < 168) {
                            solution[0] = phi1;
                            solution[1] = phi2_f_d;
                            solution[2] = phi3_f_d;
                        }
                    }
                }

            }
        }

        else if(d1 < n && -175 < phi1 < 175)                              //calculating all the backward cases
            {
                std::cout << "Fall2!! " << std::endl;
                double dpx = m - d1;                                  //calculating the x and y components of the
                std::cout << "dpx: " << dpx << std::endl;
                double dpy = wcp[2] - n;                              //distnance between the second joint and wcp
                std::cout << "dpy: " << dpy << std::endl;

                double d3 = sqrt(dpx * dpx + dpy + dpy);                  //direct distance between joint and wcp
                std::cout << "d3: " << d3 << std::endl;
                double d2 = sqrt(o * o + b * b);
                std::cout << "d2: " << d2 << std::endl;
                double beta = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)))*180/M_PI;
                std::cout << "beta: " <<  beta << std::endl;
                double alpha1 = (asin(sin(beta)*180/M_PI * (d2 / d3)))*180/M_PI;
                std::cout << "alpha1: " << alpha1 << std::endl;
                double alpha2 = (asin(dpy / d3))*180/M_PI;
                std::cout << "alpha2: " << alpha2 << std::endl;

                double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
                std::cout << "phi2_b_u: " << phi2_b_u << std::endl;
                double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down
                std::cout << "phi2_b_d: " << phi2_b_d << std::endl;

                double phi3_b_u = 270 - beta - (asin(b / d2))*180/M_PI;              //for backwards elbow up
                std::cout << "phi3_b_u: " << phi3_b_u << std::endl;
                double phi3_b_d = -1 * (90 - (beta - asin(b / d2)*180/M_PI));        //for backwards elbow down
                std::cout << "phi3_b_d: " << phi3_b_d << std::endl;

                if (-185 < phi1 < 185) {
                    if (-140 < phi2_b_u < -5) {
                        if (-120 < phi3_b_u < 168) {
                            std::cout << "phi1: " << phi1 << std::endl;
                            solution[0] = phi1;
                            solution[1] = phi2_b_u;
                            solution[2] = phi3_b_u;
                        }
                    }
                    if (-140 < phi2_b_d < -5) {
                        if (-120 < phi3_b_d < 168) {
                            std::cout << "phi1: " << phi1 << std::endl;
                            solution[0] = phi1;
                            solution[1] = phi2_b_d;
                            solution[2] = phi3_b_d;
                        }
                    }
                }
            }
        }

    else if(wcp[0]<0 && wcp[1]<0)
    {
        std::cout << "Fall3!! " << std::endl;
        phi1=180-atan(wcp[1]/wcp[0]);
        if(d1>m && wcp[2]>n && -175<phi1<175)                                                                               //calculating all the forward cases
        {
            double dpx = d1 - m;                                                                                            //calculating the x and y components of the
            double dpy = wcp[2] - n;                                                                                        //distance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                                                                    //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
            double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down

            double phi3_f_u = 360 - beta - asin(b / d2) - 90;                                                            //for forward elbow up
            double phi3_f_d = beta - asin(b / d2) - 90;                                                                  //for forward elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_f_u < -5) {
                    if (-120 < phi3_f_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_f_u;
                        solution[2] = phi3_f_u;
                    }

                    if (-140 < phi2_f_d < -5) {
                        if (-120 < phi3_f_d < 168) {
                            solution[0] = phi1;
                            solution[1] = phi2_f_d;
                            solution[2] = phi3_f_d;
                        }
                    }
                }

            }
        }
        else if(d1 < m && -175 < phi1 < 175)                              //calculating all the backward cases
        {
            double dpx = m - d1;                                  //calculating the x and y components of the
            double dpy = wcp[2] - n;                              //distnance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                  //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
            double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down

            double phi3_b_u = 270 - beta - asin(b / d2);              //for backwards elbow up
            double phi3_b_d = -1 * (90 - (beta - asin(b / d2)));        //for backwards elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_b_u < -5) {
                    if (-120 < phi3_b_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_u;
                        solution[2] = phi3_b_u;
                    }
                }
                if (-140 < phi2_b_d < -5) {
                    if (-120 < phi3_b_d < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_d;
                        solution[2] = phi3_b_d;
                    }
                }
            }
        }

    }
    else if(wcp[0]>0 && wcp[1]<0)
    {
        phi1=atan(wcp[1]/wcp[0]);
        if(d1>m && wcp[2]>n && -175<phi1<175)                                                                               //calculating all the forward cases
        {
            double dpx = d1 - m;                                                                                            //calculating the x and y components of the
            double dpy = wcp[2] - n;                                                                                        //distance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                                                                    //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
            double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down

            double phi3_f_u = 360 - beta - asin(b / d2) - 90;                                                            //for forward elbow up
            double phi3_f_d = beta - asin(b / d2) - 90;                                                                  //for forward elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_f_u < -5) {
                    if (-120 < phi3_f_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_f_u;
                        solution[2] = phi3_f_u;
                    }

                    if (-140 < phi2_f_d < -5) {
                        if (-120 < phi3_f_d < 168) {
                            solution[0] = phi1;
                            solution[1] = phi2_f_d;
                            solution[2] = phi3_f_d;
                        }
                    }
                }

            }
        }
        else if(d1 < m && -175 < phi1 < 175)                              //calculating all the backward cases
        {
            double dpx = m - d1;                                  //calculating the x and y components of the
            double dpy = wcp[2] - n;                              //distnance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                  //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
            double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down

            double phi3_b_u = 270 - beta - asin(b / d2);              //for backwards elbow up
            double phi3_b_d = -1 * (90 - (beta - asin(b / d2)));        //for backwards elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_b_u < -5) {
                    if (-120 < phi3_b_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_u;
                        solution[2] = phi3_b_u;
                    }
                }
                if (-140 < phi2_b_d < -5) {
                    if (-120 < phi3_b_d < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_d;
                        solution[2] = phi3_b_d;
                    }
                }
            }
        }
    }
    else if(wcp[0]<0 && wcp[1]>0)
    {
        phi1=180-atan(wcp[1]/-wcp[0]);
        if(d1>m && wcp[2]>n && -175<phi1<175)                                                                               //calculating all the forward cases
        {
            double dpx = d1 - m;                                                                                            //calculating the x and y components of the
            double dpy = wcp[2] - n;                                                                                        //distance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                                                                    //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_f_u = -1 * (alpha1 + alpha2);                                                                       //for forward elbow up
            double phi2_f_d = -1 * (alpha2 - alpha1);                                                                       //for forward elbow down

            double phi3_f_u = 360 - beta - asin(b / d2) - 90;                                                            //for forward elbow up
            double phi3_f_d = beta - asin(b / d2) - 90;                                                                  //for forward elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_f_u < -5) {
                    if (-120 < phi3_f_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_f_u;
                        solution[2] = phi3_f_u;
                    }

                    if (-140 < phi2_f_d < -5) {
                        if (-120 < phi3_f_d < 168) {
                            solution[0] = phi1;
                            solution[1] = phi2_f_d;
                            solution[2] = phi3_f_d;
                        }
                    }
                }

            }
        }
        else if(d1 < m && -175 < phi1 < 175)                              //calculating all the backward cases
        {
            double dpx = m - d1;                                  //calculating the x and y components of the
            double dpy = wcp[2] - n;                              //distnance between the second joint and wcp

            double d3 = sqrt(dpx * dpx + dpy + dpy);                  //direct distance between joint and wcp
            double d2 = sqrt(o * o + b * b);
            double beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
            double alpha1 = asin(sinh(beta) * (d2 / d3));
            double alpha2 = asin(dpy / d3);

            double phi2_b_u = -1 * (180 - (alpha1 + alpha2));          //for backwards elbow up
            double phi2_b_d = -1 * (180 - (alpha2 - alpha1));          //for backwards elbow down

            double phi3_b_u = 270 - beta - asin(b / d2);              //for backwards elbow up
            double phi3_b_d = -1 * (90 - (beta - asin(b / d2)));        //for backwards elbow down

            if (-185 < phi1 < 185) {
                if (-140 < phi2_b_u < -5) {
                    if (-120 < phi3_b_u < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_u;
                        solution[2] = phi3_b_u;
                    }
                }
                if (-140 < phi2_b_d < -5) {
                    if (-120 < phi3_b_d < 168) {
                        solution[0] = phi1;
                        solution[1] = phi2_b_d;
                        solution[2] = phi3_b_d;
                    }
                }
            }
        }
    }

        //Special cases
        //Special case 1:
    else if(wcp[0]==0 && wcp[1]>0)
    {
    }
        //special case 2:
    else if(wcp[0]==0 && wcp[1]<0)
    {
    }

    std::cout<< "Solution_phi1:" << solution[0];
    std::cout<< "Solution_phi2:" << solution[1];
    std::cout<< "Solution_phi3:" << solution[2];

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