#include "ptp.h"
#define _USE_MATH_DEFINES
#define PI 3.14159265
#include <iostream>

Trajectory* Ptp::get_ptp_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg)
{

    Trajectory* trajectory = new Trajectory();

    vector<Configuration*> configurations;

    //TODO: IMPLEMENT! implement the computation of a ptp trajectory with the corresponding velocity profile
    //
    //return get_ptp_trajectoy_assynch(_start_cfg, _end_cfg);
    double t_fin = 0;
    double t_c = 0;
    double max_acc = 300;

    
    double in_phi1 = _start_cfg->get_configuration()[0] * 180/ PI;

    double fin_phi1 = _end_cfg->get_configuration()[0] * 180 / PI;
    
    double max_vel_phi1 = 120;

    double distance_phi1 = abs(fin_phi1 - in_phi1);

    t_fin = max_vel_phi1 / max_acc + distance_phi1 / max_vel_phi1;
    t_c = max_vel_phi1 / max_acc;

    double in_phi2 = _start_cfg->get_configuration()[1] * 180 / PI;

    double fin_phi2 = _end_cfg->get_configuration()[1] * 180 / PI;

    double max_vel_phi2 = 115;

    double distance_phi2 = abs(fin_phi2 - in_phi2);

    if (max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2 > t_fin) {
        t_fin = max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2;
        t_c = max_vel_phi2 / max_acc;
    }
    
    double in_phi3 = _start_cfg->get_configuration()[2] * 180 / PI;

    double fin_phi3 = _end_cfg->get_configuration()[2] * 180 / PI;

    double max_vel_phi3 = 120;

    double distance_phi3 = abs(fin_phi3 - in_phi3);

    if (max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3 > t_fin) {
        t_fin = max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3;
        t_c = max_vel_phi3 / max_acc;
    }

    double in_phi4 = _start_cfg->get_configuration()[3] * 180 / PI;

    double fin_phi4 = _end_cfg->get_configuration()[3] * 180 / PI;

    double max_vel_phi4 = 190;

    double distance_phi4 = abs(fin_phi4 - in_phi4);

    if (max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4 > t_fin) {
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

    if (distance_phi1<0.1 && distance_phi2 < 0.1 && distance_phi3 < 0.1 && distance_phi4 < 0.1 && distance_phi5 < 0.1 && distance_phi6 < 0.1) {

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
    trajectory->set_trajectory(configurations);
   

    return trajectory;
}


double Ptp::trap_prof(double max_velo, double max_acc, double in_angle, double fin_angle, double time, double t_c, double t_fin) {
    
    if (abs(in_angle-fin_angle) < 0.05) {
        return(fin_angle * PI / 180);
    }
    if (time == 0) {
        return (in_angle * PI / 180);
    }
    else if (0 < time && time <= t_c) {
        
        if (in_angle < fin_angle) {
            return (in_angle + 0.5 * max_acc * time * time) * PI/180;
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

Trajectory* Ptp::get_ptp_trajectoy_assynch(Configuration* _start_cfg, Configuration* _end_cfg)
{

  
    double max_acc = 300;

    Trajectory* trajectory = new Trajectory();
    
    double in_phi1 = _start_cfg->get_configuration()[0] * 180 / PI;

    double fin_phi1 = _end_cfg->get_configuration()[0] * 180 / PI;

    double max_vel_phi1 = 120;

    double distance_phi1 = abs(fin_phi1 - in_phi1);

    double t_fin = max_vel_phi1 / max_acc + distance_phi1 / max_vel_phi1;



    double in_phi2 = _start_cfg->get_configuration()[1] * 180 / PI;

    double fin_phi2 = _end_cfg->get_configuration()[1] * 180 / PI;

    double max_vel_phi2 = 115;

    double distance_phi2 = abs(fin_phi2 - in_phi2);

    if (t_fin < max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2) {
        t_fin = max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2;
    }


 
    double in_phi3 = _start_cfg->get_configuration()[2] * 180 / PI;

    double fin_phi3 = _end_cfg->get_configuration()[2] * 180 / PI;

    double max_vel_phi3 = 120;

    double distance_phi3 = abs(fin_phi3 - in_phi3);

    if (t_fin < max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3) {
        t_fin = max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3;
    }



    double in_phi4 = _start_cfg->get_configuration()[3] * 180 / PI;

    double fin_phi4 = _end_cfg->get_configuration()[3] * 180 / PI;

    double max_vel_phi4 = 190;

    double distance_phi4 = abs(fin_phi4 - in_phi4);

    if (t_fin < max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4) {
        t_fin = max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4;
    }


    double in_phi5 = _start_cfg->get_configuration()[4] * 180 / PI;

    double fin_phi5 = _end_cfg->get_configuration()[4] * 180 / PI;

    double max_vel_phi5 = 180;

    double distance_phi5 = abs(fin_phi5 - in_phi5);

    if (t_fin < max_vel_phi5 / max_acc + distance_phi5 / max_vel_phi5) {
        t_fin = max_vel_phi5 / max_acc + distance_phi5 / max_vel_phi5;
    }



    double in_phi6 = _start_cfg->get_configuration()[5] * 180 / PI;

    double fin_phi6 = _end_cfg->get_configuration()[5] * 180 / PI;

    double max_vel_phi6 = 260;

    double distance_phi6 = abs(fin_phi6 - in_phi6);

    if (t_fin < max_vel_phi6 / max_acc + distance_phi6 / max_vel_phi6) {
        t_fin = max_vel_phi6 / max_acc + distance_phi6 / max_vel_phi6;
    }

   
    vector<Configuration*> configurations;

    if (distance_phi1 < 0.1 && distance_phi2 < 0.1 && distance_phi3 < 0.1 && distance_phi4 < 0.1 && distance_phi5 < 0.1 && distance_phi6 < 0.1) {

        configurations.push_back(_end_cfg);
        configurations.push_back(_end_cfg);
        trajectory->set_trajectory(configurations);
        return trajectory;

    }


    
    for (double i = 0; i <= t_fin; i = i + 10) {
        configurations.push_back(new Configuration({ trap_prof(max_vel_phi1 ,max_acc ,in_phi1 ,fin_phi1 ,i,max_vel_phi1 / max_acc, max_vel_phi1 / max_acc + distance_phi1 / max_vel_phi1),trap_prof(max_vel_phi2 ,max_acc ,in_phi2 ,fin_phi2 ,i,max_vel_phi2 / max_acc, max_vel_phi2 / max_acc + distance_phi2 / max_vel_phi2),trap_prof(max_vel_phi3 ,max_acc ,in_phi3 ,fin_phi3 ,i,max_vel_phi3 / max_acc, max_vel_phi3 / max_acc + distance_phi3 / max_vel_phi3),trap_prof(max_vel_phi4 ,max_acc ,in_phi4 ,fin_phi4 ,i,max_vel_phi4 / max_acc , max_vel_phi4 / max_acc + distance_phi4 / max_vel_phi4),trap_prof(max_vel_phi5 ,max_acc ,in_phi5 ,fin_phi5 ,i,max_vel_phi5 / max_acc , max_vel_phi5 / max_acc + distance_phi5 / max_vel_phi5),trap_prof(max_vel_phi6 ,max_acc ,in_phi6 ,fin_phi6 ,i,max_vel_phi6 / max_acc , max_vel_phi6 / max_acc + distance_phi6 / max_vel_phi6) }));
    }
    configurations.push_back(_end_cfg);
    trajectory->set_trajectory(configurations);
   

    return trajectory;



}