#include "spline.h"
#define _USE_MATH_DEFINES
#define PI 3.1415
#include <iostream>
#include "../../kinematics/inverse/inverse_kinematics.h"
#include "../../kinematics/direct/fw_kinematics.h"
#include "../../sdir_ctrl.h"
#include "../lin/lin.h"
#include "../ptp/ptp.h"

//steps
//take first 3 points P0,P1 and P2
//calcualate distances between them and directional velocity and acceleration unit vectors at by using P1 and P5 
//find P1,P2,P3 and P4
//send to interpolate function to find intermediate points from t=0 to t=1
//find trajectory between each points using PTP
//stitch each sets of trajectory between each sets of points
//return total trajectory

Trajectory* Spline::get_spline_trajectoy(Configuration* _start_cfg, std::vector<SixDPos> coordinates_list)
{

    //take the current position of robot and insert into the first position of coordinates list given by the user in GUI
    Configuration* spline_trajectory = new Configuration();
    Trajectory*  temp_trajectory = new Trajectory();
    vector<SixDPos*>* coordinate_list_full = new vector<SixDPos*>();
    FwKinematics fwkinematics;
    InvKinematics invkinmatics;
    SixDPos* start = fwkinematics.get_fw_kinematics(_start_cfg);
    coordinates_list.insert(coordinates_list.begin(), start);

    double P1[3], P2[3], P3[3], P4[3];
    int num_of_entry = coordinates_list.size();

    double A = coordinates_list[0]->get_A();
    double B = coordinates_list[0]->get_B();
    double C = coordinates_list[0]->get_C();


    double ts[3] ={0,0,0}, as[3] ={0,0,0};

    for (int i=0; i < coordinates_list.size(); i++)
    {
        int j = i + 1;
        int k = i + 2;
        double x0, y0, z0, x5, y5, z5, x10, y10, z10;

        double P0[3] = {coordinates_list[i]->get_X(),coordinates_list[i]->get_Y(),coordinates_list[i]->get_Z() };
        double P5[3] = {coordinates_list[j]->get_X(),coordinates_list[j]->get_Y(),coordinates_list[j]->get_Z() };
        double P10[3] = {coordinates_list[k]->get_X(),coordinates_list[k]->get_Y(),coordinates_list[k]->get_Z() };

        double Vector_P0_5[3] = {(P5[0] - P0[0]),(P5[1] - P0[1]),(P5[2] - P0[2])};
        double Vector_P5_10[3] = {(P10[0] - P5[0]),(P10[1] - P5[1]),(P10[2] - P5[2])};

        //ts, te
        
        //magnitude of ending velocity = |(vector1+vector2)|
        double mag_vel = sqrt(pow(Vector_P5_10[0] + Vector_P0_5[0], 2) + pow(Vector_P5_10[1] + Vector_P0_5[1], 2) + pow(Vector_P5_10[2] + Vector_P0_5[2], 2));
        //vector1+vector2/magnitude 
        double te[3] = { (Vector_P0_5[0] + Vector_P5_10[0]) / mag_vel, (Vector_P0_5[1] + Vector_P5_10[1]) / mag_vel, (Vector_P0_5[2] + Vector_P5_10[2]) / mag_vel };
        
        //as, ae
        
        //magnitude of change in velocity vectors from 1st to second vector
        double mag_acc = sqrt(pow(Vector_P0_5[0] + Vector_P5_10[0], 2) + pow(Vector_P0_5[1] + Vector_P5_10[1], 2) + pow(Vector_P0_5[2] + Vector_P5_10[2], 2));
        //unit acceleration vector components
        double ae[3] = { ((te[0] + ts[0]) / mag_vel),((te[1] + ts[1]) / mag_vel),((te[2] + ts[2]) / mag_vel )};

        //distance from P0 to P1
        double d0 = sqrt(pow(P5[0] - P0[0], 2) + pow(P5[1] - P0[1], 2) + pow(P5[2] - P0[2], 2));;
        //distance from P1 to P2
        double d1 = sqrt(pow(P10[0] - P5[0], 2) + pow(P10[1] - P5[1], 2) + pow(P10[2] - P5[2], 2));;

        //if second last point
        if (num_of_entry == (i + 2))
        {
            te[0] = 0; ae[0] = 0;
            te[1] = 0; ae[1] = 0;
            te[2] = 0; ae[2] = 0;
        }

        //if only 2 points it follows straight line
        else if (num_of_entry == (i + 1))
        {
            te[0] = 0; ae[0] = 0; ts[0] = 0; as[0] = 0;
            te[1] = 0; ae[1] = 0; ts[1] = 0; as[1] = 0;
            te[2] = 0; ae[2] = 0; ts[2] = 0; as[2] = 0;
        }
        else if (i ==0)
        {
            ts[0] = 0; as[0] = 0;
            ts[1] = 0; as[1] = 0;
            ts[2] = 0; as[2] = 0;
        }
        // P0[2],  P1[2],  P2[2],  P3[2],  P4[2], d0,  ts,  as, te, ae
        //x
        find_P1234(P0[0], P1[0], P2[0], P3[0], P4[0], P5[0], d0, ts[0], te[0], as[0], ae[0]);
        //y
        find_P1234(P0[1], P1[1], P2[1], P3[1], P4[1], P5[1], d0, ts[1], te[1], as[1], ae[1]);
        //z
        find_P1234(P0[2], P1[2], P2[2], P3[2], P4[2], P5[2], d0, ts[2], te[2], as[2], ae[2]);

        //now we have P1, P2, P3, P4 all (x,y and z)
        double S_x, S_y, S_z;

        for (double t = 0.000; t <= 1; t += 0.100)
        {
            //get position value S(t) from t = 0 to t=1
            S_x = Interpolate(t, P0[0], P1[0], P2[0], P3[0], P4[0], P5[0]);
            S_y = Interpolate(t, P0[1], P1[1], P2[1], P3[1], P4[1], P5[1]);
            S_z = Interpolate(t, P0[2], P1[2], P2[2], P3[2], P4[2], P5[2]);
            SixDPos* pos = new SixDPos({S_x,S_y,S_z,A,B,C});
            coordinate_list_full->push_back(pos);
        }

            for(int r = 0;r<3;r++)
            {
                ts[r] = te[r];//starting velocity vector components is set to the next line's ending
                as[r] = ae[r];//starting acceleration component is set to next lines ending
            }

    }
    //coordinate_list_full Contains list of end points.
    //Next step is motion planning


    //MOTION PLANNING TO GET TRAJCTORY
    double number_of_solutions = coordinate_list_full->size();
    Trajectory* reference = temp_trajectory;


    double x, y, z;
    vector<double> x_array;
    vector<double> y_array;
    vector<double> z_array;
    vector<Configuration*> config_2;

    for (int k = 0; k < number_of_solutions - 1; k++)
    {
                SixDPos* start = coordinate_list_full->at(k);
                SixDPos* end = coordinate_list_full->at(k+1);
                ////////////////// ///////////////////////////////////////////////////////////////
                ///        LIN CODE TO FIND THE BEST ROUTE
                ///////////////////////////
                double t_fin_x = 0;
                double t_fin_y = 0;
                double t_fin_z = 0;
                double t_fin = 0;
                double t_c = 0;
                double max_acc = 1;//(m/s)
                double max_vel = 2;//(m/s^2)

                Trajectory* trajectory = new Trajectory();
                FwKinematics fwkinematics;
                InvKinematics inversekinematics;

                ////SixDPos* start;
                ////SixDPos* end;
                //start = fwkinematics.get_fw_kinematics(_start_cfg);
                //end = fwkinematics.get_fw_kinematics(_end_cfg);
                //double a = start[0]
        
                vector<double> in;
                vector<double> fin;
                in[0] = start->get_X();//in_x
                fin[0] = end->get_X();//in_x1
                in[1] = start->get_Y();//in_y
                fin[1] = end->get_Y();//in_y1
                in[2] = start->get_Z();//in_z
                fin[2] = end->get_Z();//in_z1
                in[3] = start->get_A();//a
                in[4] = start->get_B();//b
                in[5] = start->get_C();//c
                fin[3] = end->get_A();//a1
                fin[4] = end->get_B();//b1
                fin[5] = end->get_C();//c1

                //Step-3:
                t_c = max_vel / max_acc;
                //to get t_fin we need fin-in,t_fin =t_c+(fin-in)/max_vel
                //common distance
                t_fin_x = t_c + (fin[0] - in[0]) / max_vel;
                t_fin_y = t_c + (fin[1] - in[1]) / max_vel;
                t_fin_z = t_c + (fin[2] - in[2]) / max_vel;
                if (t_fin_x > t_fin_y) {
                    if (t_fin_x > t_fin_z) {
                        t_fin = t_fin_x;
                    }
                    else {
                        t_fin = t_fin_z;
                    }
                }
                else {
                    if (t_fin_y > t_fin_z) {
                        t_fin = t_fin_y;
                    }
                    else {
                        t_fin = t_fin_z;
                    }
                }

                //i = time cycles
                for (double i = 0; i <= t_fin; i = i + 0.5) {
                    x = compute_1(max_vel, max_acc, in[0], fin[0], i, t_c, t_fin);
                    y = compute_1(max_vel, max_acc, in[1], fin[1], i, t_c, t_fin);
                    z = compute_1(max_vel, max_acc, in[2], fin[2], i, t_c, t_fin);
                    x_array[i] = x;
                    y_array[i] = y;
                    z_array[i] = z;

                    SixDPos* position_and_orientation;
                    vector<SixDPos*> SixDPos_first;
                    vector<SixDPos*> SixDPos_second;
                    vector<Configuration*>* config_1 = new vector<Configuration*>();
                    vector<Configuration*>* config_1_2 = new vector<Configuration*>();


                    SixDPos_first.push_back(new SixDPos(x_array[0], y_array[0], z_array[0], in[3], in[4], in[5]));
                    SixDPos_second.push_back(new SixDPos(x_array[1], y_array[1], z_array[1], in[3], in[4], in[5]));

                    if (inversekinematics.get_inv_kinematics(new SixDPos(x_array[0], y_array[0], z_array[0], in[3], in[4], in[5])) == nullptr)
                    {
                        //Where do you check for singularities?? overhead needs to be handled by you, discuss with sreenath what value to excpect for singularities
                        std::cout << "'No solution returned from Inverse Kinematics";
                    }
                    else
                    {
                        SixDPos* pos = new SixDPos({ x_array[i], y_array[i], z_array[i], in[3], in[4], in[5] });
                        config_1 = inversekinematics.get_inv_kinematics(pos);
                        int max_distance = 2701;
                        int temporary_distance_start;
                        int temporary_distance;
                        int temporary_difference;
                        vector<double> distance;
                        int solution_number = 0;
                        int best_solution_number = 0;
                        double number_of_solutions = config_1->size();
                        cout << number_of_solutions;
                        if (i == 0)
                        {
                            for (int j = 0; j < number_of_solutions; j++)
                            {
                                solution_number = j;
                                //cout <<"\n FLAGGGGGGG\n"<<" **"<<_start_cfg<< " **" << _start_cfg->get_configuration().at(1) << " **" <<_start_cfg->get_configuration().at(2) << " **" <<_start_cfg->get_configuration().at(3) << " **" << _start_cfg->get_configuration().at(4) <<" **"<< _start_cfg->get_configuration().at(5);
                                temporary_distance_start = _start_cfg->get_configuration().at(0) + _start_cfg->get_configuration().at(1) + _start_cfg->get_configuration().at(2) + _start_cfg->get_configuration().at(3) + _start_cfg->get_configuration().at(4) + _start_cfg->get_configuration().at(5);
                                temporary_distance = config_1->at(solution_number)->operator[](0) + config_1->at(solution_number)->operator[](1) + config_1->at(solution_number)->operator[](2) + config_1->at(solution_number)->operator[](3) + config_1->at(solution_number)->operator[](4) + config_1->at(solution_number)->operator[](5);
                                cout << "\ntemp dist: " << temporary_distance;
                                temporary_difference = temporary_distance_start - temporary_distance;
                                cout << "\ntemp dist: " << temporary_difference;
                                //solution_number++;
                                if (temporary_difference < max_distance)
                                {
                                    max_distance = temporary_difference;
                                    best_solution_number = solution_number;
                                }
                            }

                        }

                        config_2.push_back(config_1->at(best_solution_number));
                        config_1_2->push_back(config_1->at(best_solution_number));

                        cout << "\n Best Solution no:" << best_solution_number;

                        if (i > 0)
                        {

                            number_of_solutions = config_1->size();
                            for (int j = 0; j < number_of_solutions; j++)
                            {
                                solution_number = j;
                                temporary_distance_start = config_1_2->at(i - 1)->operator[](0) + config_1_2->at(i - 1)->operator[](1) + config_1_2->at(i - 1)->operator[](2) + config_1_2->at(i - 1)->operator[](3) + config_1_2->at(i - 1)->operator[](4) + config_1_2->at(i - 1)->operator[](5);
                                temporary_distance = config_1->at(solution_number)->operator[](0) + config_1->at(solution_number)->operator[](1) + config_1->at(solution_number)->operator[](2) + config_1->at(solution_number)->operator[](3) + config_1->at(solution_number)->operator[](4) + config_1->at(solution_number)->operator[](5);
                                temporary_difference = temporary_distance_start - temporary_distance;
                                //solution_number++;
                                if (temporary_difference < max_distance)
                                {
                                    max_distance = temporary_difference;
                                    best_solution_number = solution_number;
                                }
                                //solution_number++;  
                            }
                            config_2.push_back(config_1->at(best_solution_number));
                            config_1_2->push_back(config_1->at(best_solution_number));
                            std::cout << config_1->at(best_solution_number)->get_configuration()[0]<<" ";
                            std::cout << config_1->at(best_solution_number)->get_configuration()[1] << " ";
                            std::cout << config_1->at(best_solution_number)->get_configuration()[2] << " ";
                            std::cout << config_1->at(best_solution_number)->get_configuration()[3] << " ";
                            std::cout << config_1->at(best_solution_number)->get_configuration()[4] << " ";
                            std::cout << config_1->at(best_solution_number)->get_configuration()[5]<<endl;
                        }
                    }

                }
    }
////////////////////////////////////////////////////////////////
    //covert config2 to trajectory

    temp_trajectory->set_trajectory(config_2);
    return temp_trajectory;
}

void Spline::find_P1234(double P0, double P1, double P2, double P3, double P4, double P5, double d0, double ts, double as, double te, double ae)
{

    //consideting P1 and P4 and affected by distance between P0 and P5 and the direction of velocity at P5
    P4 = P5 - (0.2 * d0 * te); 
    P3 = (0.05 * ae) + 2 * P4 - P5; //
    P1 = P0 + (0.2 * d0 * ts);
    P2 = (0.05 * as) + (2 * P1) - P0;
}


double Spline::Interpolate(double t, double P0, double P1, double P2, double P3, double P4, double P5)
{
    double S=0;
    double T[6] = { pow(t,5),pow(t,4),pow(t,3),pow(t,2),pow(t,1),1.000 };
    double CQB[6] = {     (-P0 + (5 * P1) - (10 * P2) + (10 * P3) - (5 * P4) + (P5)),
                          ((5 * P0) - (20 * P1) + (30 * P2) - (20 * P3) + (5 * P4)),
                          ((-10 * P0) + (30 * P1) - (30 * P2) + (10 * P3)),
                          ((10 * P0) - (20 * P1) + (10 * P2)),
                          ((-5 * P0 + (5 * P1))),
                          (P0)
                    };


    for (int i = 0; i < 6; i++)
    {
        S += T[i] * CQB[i];
    }
    
    return S;
}

double Spline::compute_1(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin) {
    // you forgot the time = t_c, time = t_fin-t_c and time = t_fin cases, so in those cases the function wouldnt return anything which causes an error.
    // NOTE!!! check my implementation of ptp, you need to check if you are moving in a positiv or negativ direction, otherwise you might end up going in the other direction
    if (time <= t_c) {
        return (in_angle + 0.5 * max_acc * time * time);
        //call inversekinematics
    }
    if (t_c < time <= t_fin - t_c) {
        return (in_angle + max_acc * t_c * (time - t_c / 2));
    }
    if (t_fin - t_c < time <= t_fin) {
        return (fin_angle - 0.5 * max_acc * (t_fin - time) * (t_fin - time));
    }
}