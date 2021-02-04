#include "spline.h"
#define _USE_MATH_DEFINES
#define PI 3.1415
#include <iostream>
#include "../../kinematics/inverse/inverse_kinematics.h"
#include "../../kinematics/direct/fw_kinematics.h"
#include "../../sdir_ctrl.h"

//steps
//1. Get current position and vector of configurations from the main program
//2. Set the robot to find the first point from the list of coordinates entered by user
//3. Send both the coordinates to find the best final configuration
//4. Send the configs to "get_velocity" function to find the maximum velocity and acceleration of end effector
//5. This value is assigned to te and ae 
//6. Set of (x0,x5) which (p0 and p5) is sent to "interpolate" function to find p1,p2,p3 and p4
//7. After inputting ts =0, as =0 (initial velocity and acceleration is assumed to be 0).......
//.....te and ae as returned velocity and acceleration
//8. Matrices of [t5,t4,...] is multiplied with coefficient matrix (functions of P0,P1,P2....)
//9. Now we iterate value of t=0.1 to t=1.0 to get 10 sets of trajectory points.
//10. this trajectory is only with either an x or y or z
//11. similarly send y and z to interpolate
//    11.1.  make the first point as previous p5
//    11.2.  make p5 as next point
//    11.3.  repeat

//12. stitch together the points on the "get_splie_trajectory" function and push to 'spline_trajectory'
//13. push back the trajectory

Trajectory* Spline::get_spline_trajectoy(Configuration* _start_cfg, std::vector<SixDPos*> coordinates_list)
{

    //function that receives a vector of Six D positions and the current configuration of the robot
    
    Trajectory* spline_trajectory = new Trajectory();
    SdirCtrl ctrl;
    SixDPos* start;
    SixDPos* first_point;
    FwKinematics* fwkinematics;
    double ts, te, as, ae;
    ts= te= as= ae=0.000;


//// get first point value and current point value both in SixDPos format


    start = fwkinematics->get_fw_kinematics(_start_cfg);

    ////move to the first set of number in LIN motion and get this trajectory
    //spline_trajectory = ctrl.move_robot_lin(start, first_point);

    //starting is the current position
    SixDPos P0 = start;
   // this will cause an error, you are initializing the same points over and over again, not sure what the goal here is
   // is P0_x.... supposed to be an array? 
    for(int i = 0; i < coordinates_list.size(); i++)
    {

        SixDPos P1, P2, P3, P4;

        SixDPos P5 = coordinates_list[i]->get_position();
        
        //find coordinates x,y,z of first control point
        double P0_x = P0[0];
        double P0_y = P0[1];
        double P0_z = P0[2];

        //find coordinates x,y,z of final control point
        double P5_x = P5[0];
        double P5_y = P5[1];
        double P5_z = P5[2];


        //FIND THE BEST CONFIGURATION TO MOVE BETWEEN TWO POINTS
        Configuration *cfg1, *cfg2;
        find_best_cfg(P0, P5, cfg1, cfg2);
        //SEND THESE CONFIG TO THE get_velocity function to FIND the max VELOCITY and acceleration
        std::tie(as, ae) = get_velocity(cfg1, cfg2);


        //function for x
        vector<double> pathx;
        interpolate(P0[0], P5[0], pathx, ts, te, as, ae );

        vector<double> pathy;
        interpolate(P0[1], P5[1], pathy, ts, te, as, ae);

        vector<double> pathz;
        interpolate(P0[2], P5[2], pathz, ts, te, as, ae);
        //call fucntion to find y

        for (int k = 0; k < pathx.size(); k++)
        {
            spline_trajectory->add_configuration({ /*push 3 values }*/ });
        } //orientation must be start orientation

        
        SixDPos P0 = P5;
        ts = te;
        as = ae;
        
    }
             
 return spline_trajectory;
}


void Spline::interpolate(double p0, double p5, vector<double> &path, double ts, double te, double as, double ae)
{
        //pass as pointer and pushback path*, no need to use vector here, double* would work fine 
        vector<double>* temp = &path;
        //intermediate control points
        double p1, p2, p3, p4, s;

        double t = 0.5;

        p1 = (  (1/5)*ts   ) + p0;
        p2 = ((1 / 20) * as) + (2 * p1) - p0;
        p4 = p5 - ((1 / 5) * te);
        p3 = ((1 / 20) * ae) + (2 * p4) - (p5);
        
        for (t = 0.1; t <= 1; t+=0.1)
        {
            double T[1][6] = { pow(t,5),pow(t,4),pow(t,3),pow(t,2),pow(t,1),1.000 };
            double CQB[6][1] = { {-p0 + (5 * p1) - (10 * p2) + (10 * p3) - (5 * p4) + (p5)},
                                 {(5 * p0) - (20 * p1) + (30 * p2) - (20 * p3) + (5 * p4)},
                                  {(-10 * p0) + (30 * p1) - (30 * p2) + (10 * p3)},
                                  {(10 * p0) - (20 * p1) + (10 * p2)},
                                  {(-5 * p0 + (5 * p1))},
                                  {p0} };

            double S = S_Functon(T, CQB);
            temp->push_back(s);
        }
 }

 double Spline::S_Functon(double M1[1][6], double M2[6][1])
 {
       //net rotational matrix 0R3 is product of first and second rotational matrix

        double sum;
        for (int i = 0; i < 6; i++)
        {
            sum += M1[1][i] * M2[i][1];
        }
        return sum;
 }
 //not sure why you need all this ptp code if you just need max_velo
 std::tuple<double, double> Spline::get_velocity(Configuration *_start_cfg, Configuration *_end_cfg)
{

    double t_fin = 0;
    double t_c = 0;
    double max_acc = 300;

    double in_phi1 = _start_cfg->get_configuration()[0] * 180 / PI;
    double fin_phi1 = _end_cfg->get_configuration()[0] * 180 / PI;
    double max_vel_phi1 = 120;
    double distance_phi1 = abs(fin_phi1 - in_phi1);

    t_fin = max_vel_phi1 / max_acc + distance_phi1 / max_vel_phi1;
    t_c = max_vel_phi1 / max_acc;


    double in_phi2 = _start_cfg->get_configuration()[1] * 180 / PI;
    double fin_phi2 = _end_cfg->get_configuration()[1] * 180 / PI;
    double max_vel_phi2 = 115;
    double distance_phi2 = abs(fin_phi2 - in_phi2);

    if((max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2) > t_fin) 
    {
        t_fin = max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2;
        t_c = max_vel_phi2 / max_acc;
    }

    double in_phi3 = _start_cfg->get_configuration()[2] * 180 / PI;

    double fin_phi3 = _end_cfg->get_configuration()[2] * 180 / PI;

    double max_vel_phi3 = 120;

    double distance_phi3 = abs(fin_phi3 - in_phi3);

    if((max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3) > t_fin)
    {
        t_fin = max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3;
        t_c = max_vel_phi3 / max_acc;
    }

    double in_phi4 = _start_cfg->get_configuration()[3] * 180 / PI;
    double fin_phi4 = _end_cfg->get_configuration()[3] * 180 / PI;
    double max_vel_phi4 = 190;
    double distance_phi4 = abs(fin_phi4 - in_phi4);

    if(max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4 > t_fin) {
        t_fin = max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4;
        t_c = max_vel_phi4 / max_acc;
    }

    double in_phi5 = _start_cfg->get_configuration()[4] * 180 / PI;

    double fin_phi5 = _end_cfg->get_configuration()[4] * 180 / PI;

    double max_vel_phi5 = 180;

    double distance_phi5 = abs(fin_phi5 - in_phi5);

    if (max_vel_phi5 / max_acc + distance_phi5 / max_vel_phi5 > t_fin) {
        t_fin = max_vel_phi5 / max_acc + distance_phi5 / max_vel_phi5;
        t_c = max_vel_phi5 / max_acc;
    }

    double in_phi6 = _start_cfg->get_configuration()[5] * 180 / PI;

    double fin_phi6 = _end_cfg->get_configuration()[5] * 180 / PI;

    double max_vel_phi6 = 260;

    double distance_phi6 = abs(fin_phi6 - in_phi6);

    if (max_vel_phi6 / max_acc + distance_phi6 / max_vel_phi6 > t_fin) {
        t_fin = max_vel_phi6 / max_acc + distance_phi6 / max_vel_phi6;
        t_c = max_vel_phi6 / max_acc;
    }

    max_vel_phi1 = distance_phi1 / (t_fin - t_c);
    double max_acc1 = max_vel_phi1 / t_c;

    max_vel_phi2 = distance_phi2 / (t_fin - t_c);
    double max_acc2 = max_vel_phi2 / t_c;

    max_vel_phi3 = distance_phi3 / (t_fin - t_c);
    double max_acc3 = max_vel_phi3 / t_c;

    max_vel_phi4 = distance_phi4 / (t_fin - t_c);
    double max_acc4 = max_vel_phi4 / t_c;

    max_vel_phi5 = distance_phi5 / (t_fin - t_c);
    double max_acc5 = max_vel_phi5 / t_c;

    max_vel_phi6 = distance_phi6 / (t_fin - t_c);
    double max_acc6 = max_vel_phi6 / t_c;

    /*if (distance_phi1 < 0.1 && distance_phi2 < 0.1 && distance_phi3 < 0.1 && distance_phi4 < 0.1 && distance_phi5 < 0.1 && distance_phi6 < 0.1) {

        configurations.push_back(_end_cfg);
        configurations.push_back(_end_cfg);
        trajectory->set_trajectory(configurations);
        return trajectory;

    }


    std::cout << _start_cfg->get_configuration()[0] << "  " << _start_cfg->get_configuration()[1] << "  " << _start_cfg->get_configuration()[2] << "  " << _start_cfg->get_configuration()[3] << "  " << _start_cfg->get_configuration()[4] << "  " << _start_cfg->get_configuration()[5] << endl;
    std::cout << _end_cfg->get_configuration()[0] << "  " << _end_cfg->get_configuration()[1] << "  " << _end_cfg->get_configuration()[2] << "  " << _end_cfg->get_configuration()[3] << "  " << _end_cfg->get_configuration()[4] << "  " << _end_cfg->get_configuration()[5] << endl;


    for (double i = 0; i <= t_fin; i = i + 0.1) {
        configurations.push_back(new Configuration({ trap_prof(max_vel_phi1 ,max_acc1 ,in_phi1 ,fin_phi1 ,i,t_c, t_fin),trap_prof(max_vel_phi2 ,max_acc2 ,in_phi2 ,fin_phi2 ,i,t_c, t_fin),trap_prof(max_vel_phi3 ,max_acc3 ,in_phi3 ,fin_phi3 ,i,t_c, t_fin),trap_prof(max_vel_phi4 ,max_acc4 ,in_phi4 ,fin_phi4 ,i,t_c, t_fin),trap_prof(max_vel_phi5 ,max_acc5 ,in_phi5 ,fin_phi5 ,i,t_c, t_fin),trap_prof(max_vel_phi6 ,max_acc6 ,in_phi6 ,fin_phi6 ,i,t_c, t_fin) }));

        std::cout << trap_prof(max_vel_phi1, max_acc1, in_phi1, fin_phi1, i, t_c, t_fin) << "   " << trap_prof(max_vel_phi2, max_acc2, in_phi2, fin_phi2, i, t_c, t_fin) << "   " << trap_prof(max_vel_phi3, max_acc3, in_phi3, fin_phi3, i, t_c, t_fin) << "  " << trap_prof(max_vel_phi4, max_acc4, in_phi4, fin_phi4, i, t_c, t_fin) << "     " << trap_prof(max_vel_phi5, max_acc5, in_phi5, fin_phi5, i, t_c, t_fin) << "   " << trap_prof(max_vel_phi6, max_acc6, in_phi6, fin_phi6, i, t_c, t_fin) << endl << endl;
    }
    configurations.push_back(_end_cfg);
    trajectory->set_trajectory(configurations);*/


    return make_tuple(max_vel_phi6, max_acc6);


}
double Spline::trap_prof(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin) {

    if (abs(in_angle - fin_angle) < 0.05) {
        return(fin_angle * PI / 180);
    }
    if (time == 0) {
        return (in_angle * PI / 180);
    }
    else if (0 < time && time <= t_c) {

        if (in_angle < fin_angle) {
            return (in_angle + 0.5 * max_acc * time * time) * PI / 180;
        }
        if (in_angle > fin_angle) {
            return (in_angle - 0.5 * max_acc * time * time) * PI / 180;
        }
    }
    else if (t_c < time && time <= t_fin - t_c) {

        if (in_angle < fin_angle) {
            return(in_angle + max_acc * t_c * (time - t_c / 2)) * PI / 180;
        }
        if (in_angle > fin_angle) {
            return(in_angle - max_acc * t_c * (time - t_c / 2)) * PI / 180;
        }
    }
    else if (t_fin - t_c < time && time <= t_fin) {

        if (in_angle < fin_angle) {
            return (fin_angle - 0.5 * max_acc * (t_fin - time) * (t_fin - time)) * PI / 180;
        }
        if (in_angle > fin_angle) {
            return (fin_angle + 0.5 * max_acc * (t_fin - time) * (t_fin - time)) * PI / 180;
        }
    }
}
void Spline:: find_best_cfg(SixDPos P0, SixDPos P5, Configuration *cfg1, Configuration *cfg2)
{


}