#include "ptp.h"
#include<iostream>
#include "ConfigProvider.h"


Trajectory* Ptp::get_ptp_trajectoy(Configuration* _start_cfg, Configuration* _end_cfg, bool sync)
{
    //TODO: IMPLEMENT! implement the computation of a ptp trajectory with the corresponding velocity profile
    // notes: point to point is from initial position to final position,
    // while velocity, acceleration function should be continuous
    // velocity profile has constant acceleration, deceleration, velocity
    std::cout << "------------------------PTP----------------------------" << std::endl;
    Trajectory* trajectory = new Trajectory();

    Trajectory res;
    vector<Configuration*> config_vec;
    vector<Configuration*> config_vec2;
    double M_PII = 3.14159;
    double a_max = ConfigProvider::getInstance().getMax_accel();     // max acceleration
    double timesteps = ConfigProvider::getInstance().getsteps_per_second(); // 0.001s; so r = 1000 = 1s


    // initialize all the arrays
//    array<double, 6> array_startpos{};
//    array<double, 6> array_endpos{};
    array<double, 6> array_t_pos{};
    array<double, 6> array_v_max{};
    array<double, 6> array_v_adjust{};
    array<double, 6> array_a_adjust{};

    std::vector<double> joint_startpos_vec, joint_endpos_vec, joint_distance_vec;
    std::vector<double> joint_q_m_vec;
    std::vector<double> t_f_vec, t_m_vec, t_c_vec;
    std::vector<bool> joint_max_vel_profile_vec, joint_trapez_profile_vec;
    std::vector<double> t_m_max_vel_vec;

    double joint1_v_max = ConfigProvider::getInstance().getJoint1_max_vel();     //max speed of joint 1 in °/s
    double joint2_v_max = ConfigProvider::getInstance().getJoint2_max_vel();     //max speed of joint 2 in °/s
    double joint3_v_max = ConfigProvider::getInstance().getJoint3_max_vel();     //max speed of joint 3 in °/s
    double joint4_v_max = ConfigProvider::getInstance().getJoint4_max_vel();     //max speed of joint 4 in °/s
    double joint5_v_max = ConfigProvider::getInstance().getJoint5_max_vel();     //max speed of joint 5 in °/s
    double joint6_v_max = ConfigProvider::getInstance().getJoint6_max_vel();     //max speed of joint 6 in °/s


    array_v_max[0] = joint1_v_max;
    array_v_max[1] = joint2_v_max;
    array_v_max[2] = joint3_v_max;
    array_v_max[3] = joint4_v_max;
    array_v_max[4] = joint5_v_max;
    array_v_max[5] = joint6_v_max;

    for (int i = 0; i < 6; ++i) {
        // factor of velocity adjustment (for synchronised)
        array_v_adjust[i] = 1;
        // factor of acceleration adjustment (for synchronised)
        array_a_adjust[i] = 1;
    }




    for (int i = 0; i < 6; ++i) {
        // read start and endpos to vector
        joint_startpos_vec.push_back(_start_cfg->operator[](i));
        joint_endpos_vec.push_back(_end_cfg->operator[](i));

        // calculate joint distance (in degree)
        joint_distance_vec.push_back(_end_cfg->operator[](i) - _start_cfg->operator[](i));

        // calculate q_m
        joint_q_m_vec.push_back(abs(joint_distance_vec.at(i))/2);

        //calculate t_c
        t_c_vec.push_back(array_v_max[i] / a_max);

        // calculate t_m for max vel profile
        t_m_max_vel_vec.push_back(sqrt(2 * abs(joint_q_m_vec.at(i) / a_max)));

        // check which profile we need for which joint
        if (t_c_vec.at(i) >= t_m_max_vel_vec.at(i)){
            t_f_vec.push_back(2 * t_m_max_vel_vec.at(i));
            t_m_vec.push_back(t_m_max_vel_vec.at(i));
            std::cout << "We need max_vel_profile for joint " << i+1 << "." << std::endl;
            joint_max_vel_profile_vec.push_back(true);
            joint_trapez_profile_vec.push_back(false);
        }else if (t_c_vec.at(i) < t_m_max_vel_vec.at(i)){
            t_f_vec.push_back(t_c_vec.at(i) + abs(joint_distance_vec.at(i)) / array_v_max[i] );
            t_m_vec.push_back(t_f_vec.at(i) / 2);

            std::cout << "We need trapezoidal profile for joint " << i+1 << "." << std::endl;
            joint_max_vel_profile_vec.push_back(false);
            joint_trapez_profile_vec.push_back(true);
        }else{
            std::cout << "Error determining case for Joint " << i+1 << "." << std::endl;
        }
    }

    std::vector<double> joint_t_pos_vec;
    joint_t_pos_vec.reserve(6);
    for (int i = 0; i < 6; ++i) {
        joint_t_pos_vec.push_back(joint_startpos_vec.at(i));
    }
    // find largest t_f of the 6 joints
    double max_t_f = 0;
    // double max_t_f = joint1_t_f;
    for (int i = 0; i < 6; ++i) {
        std::cout << "joint1_t_f for joint " << i+1 << ": " <<  t_f_vec.at(i) << std::endl;
        if (t_f_vec.at(i) > max_t_f){
            max_t_f = t_f_vec.at(i);
        }
    }
    std::cout << "max_t_f :" << max_t_f << std::endl;


    // if bool sync == false then we need asynchron motion
    if(!sync){
        std::cout << "asynchron motion: " << std::endl;
        for (int r = 0; r <= (max_t_f * timesteps); ++r) { //r = 1 will be 0.00001s; so r = 100000 = 1s
            std::cout << "r: " << r << std::endl;
            double t = double(r) / timesteps;
            std::cout << "t: " << t << std::endl;

            for (int i = 0; i < 6; ++i) {
                if ( joint_max_vel_profile_vec.at(i) ) {
                    joint_t_pos_vec.at(i) = res.max_vel_profile(joint_startpos_vec.at(i), joint_distance_vec.at(i), t_f_vec.at(i), t_m_vec.at(i), t,
                                        joint_t_pos_vec.at(i), a_max, array_v_max[i]);
                }else if ( joint_trapez_profile_vec.at(i) ) {
                    joint_t_pos_vec.at(i) = res.trapezoidal_profile(joint_startpos_vec.at(i), joint_distance_vec.at(i), joint_endpos_vec.at(i),
                                                                    t_f_vec.at(i), t, joint_t_pos_vec.at(i), a_max, array_v_max[i]);
                }else{
                    std::cout << "ERROR: No velocity profile selected for joint " << i+1 << "." << std::endl;
                }
                std::cout << "Async: joint_t_pos for joint " << i +1 << ": " <<  joint_t_pos_vec.at(i) << std::endl;
                array_t_pos[i] = joint_t_pos_vec.at(i) * M_PII / 180;
            }

            // push Configuration into vector
            config_vec.push_back(new Configuration(array_t_pos));

        }
    } else if(sync){        // we need a synchronised motion
        // we need to slow down all trajectories except the ones with the longest t_f
        //initialize constants
        double sync_distance;
        double sync_acceleration;
        double sync_max_velocity;
        //add all start points to one vec

        for (int i = 0; i <6; i++){
            if(t_f_vec.at(i) == max_t_f) {
                // overwrite t_c (t_c of all joints should be equal to the slowest joint's t_c)
                for (int j = 0; j < 6; ++j) {
                    std::cout << "sync: before adjustment t_c[" << j+1 << "]: " << t_c_vec.at(j) << std::endl;
                    t_c_vec.at(j) = t_c_vec.at(i);
                    std::cout << "sync: after adjustment t_c[" << j+1 << "]: " << t_c_vec.at(j) << std::endl;
                }

                // set variables of slowest joint to a new fixed variable
                // (we need these for calculating the acceleration and velocity of the other joints)
                sync_distance = joint_distance_vec.at(i);
                sync_acceleration = a_max;      // slowest joint uses the maximum acceleration
                sync_max_velocity = array_v_max.at(i);
            }
        }
        for (int i = 0; i < 6; i++){
            if(t_f_vec.at(i) == max_t_f) {
                // this joint is already slowed down enough
            } else if(t_f_vec.at(i) != max_t_f) {
                double vel_constant = 0;        // constant to adjust max_velocity of joint
                double accel_constant = 0;      // constant to adjust max_acceleration of joint

                // basically calculates factor by which we need to decrease the max_velocity of the joint
                vel_constant = ( abs(joint_distance_vec.at(i)) * sync_max_velocity ) / ( abs(sync_distance) * array_v_max.at(i) );
                std::cout << "distance2: " << abs(joint_distance_vec.at(i)) << ". sync_max_vel: " << sync_max_velocity << ". sync_distance: " << sync_distance << ". v_max: " << array_v_max.at(i) << std::endl;
                std::cout << "velocity constant for joint " << i+1 << " : " << vel_constant << std::endl;
                // handle limits of the robot
                if (vel_constant > 1.0){       // has to be 0 <= constant <= 1
                    vel_constant = 1;
                    std::cout << "velocity constant for joint" << i+1 << " overwritten (check of limits):" << vel_constant << std::endl;
                    // for this case it does not work to have all joints be at max_velocity at the same time (as far as i know)
                    // so in this case we will determine a new t_c from the following
                    t_c_vec.at(i) = max_t_f - abs(joint_distance_vec.at(i))/array_v_max.at(i);
                    std::cout << "robot limits exceeded: new t_c " << i+1 << " overwritten: " << t_c_vec.at(i) << std::endl;
                }else if(vel_constant <= 0){
                    std::cout << "velocity constant <= 0 !!" << std::endl;
                }
                //add vel_constant to array
                array_v_adjust.at(i) = vel_constant;

                // basically calculates factor by which we need to decrease the acceleration of the joint
                accel_constant = (vel_constant * array_v_max.at(i))/(a_max * t_c_vec.at(i) );
                std::cout << "acceleration constant for joint" << i+1 << " :" << accel_constant << std::endl;

                // add accel_constant to array
                array_a_adjust.at(i) = accel_constant;

                // check if we get to the desired t_f with the new constants
                double check_t_f = ( ( vel_constant * array_v_max.at(i) )/ (accel_constant * a_max) )
                        + abs(joint_distance_vec.at(i))/(vel_constant * array_v_max.at(i) );



                std::cout << "new t_f for joint " << i+1 << " = " << check_t_f << ". t_f: " << max_t_f << std::endl;


            }
        }
        std::cout << "synchronised motion ptp: " << std::endl;
        for(int r = 0; r <= max_t_f*timesteps; r++){
            std::cout << "r: " << r << std::endl;
            double t = double(r) / timesteps;
            std::cout << "t: " << t << std::endl;

            for(int i = 0; i < 6; i++){
                joint_t_pos_vec.at(i) = res.trapezoidal_profile(joint_startpos_vec.at(i), joint_distance_vec.at(i), joint_endpos_vec.at(i),
                                                            t_f_vec.at(i), t, joint_t_pos_vec.at(i), array_a_adjust.at(i)*a_max, array_v_adjust.at(i)*array_v_max.at(i));
                std::cout << "t_pos for joint" << i+1 << " = " << joint_t_pos_vec.at(i) << std::endl;

                //change degree to radian
                joint_t_pos_vec.at(i) = joint_t_pos_vec.at(i) * M_PII / 180;
            }

            config_vec.push_back(new Configuration(
                    {joint_t_pos_vec.at(0), joint_t_pos_vec.at(1), joint_t_pos_vec.at(2),
                     joint_t_pos_vec.at(3), joint_t_pos_vec.at(4), joint_t_pos_vec.at(5)}));
        }


    }
    // calculate velocities!!!!
    vector<vector<double>> velocity_print;
    for (int i = 0; i < config_vec.size(); ++i) {
        if (i == 0){
            velocity_print.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        }else {
            vector<double> temp;
            temp.reserve(6);
            for (int j = 0; j < 6; ++j) {
                double dist = config_vec.at(i)->get_configuration().operator[](j) -
                              config_vec.at(i - 1)->get_configuration().operator[](j);
                temp.push_back( (dist / (1 / timesteps))*180/M_PII );
            }
            velocity_print.push_back(temp);
        }

    }
    // for Presentation purposes: Printing of velocities of each joint possible here
//    for (int i = 0; i < velocity_print.size(); ++i) {
//        std::cout << velocity_print.at(i).at(0) << std::endl;
//        std::cout << velocity_print.at(i).at(1) << std::endl;
//        std::cout << velocity_print.at(i).at(2) << std::endl;
//        std::cout << velocity_print.at(i).at(3) << std::endl;
//        std::cout << velocity_print.at(i).at(4) << std::endl;
//        std::cout << velocity_print.at(i).at(5) << std::endl;
//    }

    trajectory->set_trajectory(config_vec);



    return trajectory;
}
