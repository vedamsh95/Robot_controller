//
// Created by Timo on 07.01.2021.
//

#include "spline.h"
#include "Vector.h"
#include "math.h"
#include <iostream>
#include "Trajectory.h"



Spline::Spline(Vector<double, 3> start_point, Vector<double, 3> start_orientation,
               std::vector<Vector<double, 3>> *points, double speed, double acceleration, Vector<double, 6> start_config_vec) {

    this->start_position[0] = start_point[0];
    this->start_position[1] = start_point[1];
    this->start_position[2] = start_point[2];

    this->start_orientation[0] = start_orientation[0];
    this->start_orientation[1] = start_orientation[1];
    this->start_orientation[2] = start_orientation[2];
    //this->start_orientation[2] = 0;

    this->start_config[0] = start_config_vec[0];
    this->start_config[1] = start_config_vec[1];
    this->start_config[2] = start_config_vec[2];

    // catch wrist singularity case
    if ((start_config_vec[4]*180/M_PII) >= - marginpoint && (start_config_vec[4]*180/M_PII) <= marginpoint){
        //we have a wrist singularity
        this->start_config[3] = 0;
        this->start_config[4] = 0;
        this->start_config[5] = 0;
    }else{
        this->start_config[3] = start_config_vec[3];
        this->start_config[4] = start_config_vec[4];
        this->start_config[5] = start_config_vec[5];
    }


    this->speed = speed;
    this->acceleration = acceleration;

    this->points = points;
    this->num_points = points->size();
}

void Spline::out() {
    std::cout << "Spline is defined: " << std::endl;
    std::cout << "Start Position: " << std::endl;
    start_position.output();
    std::cout << "Current Orientation " << std::endl;
    start_orientation.output();
    std::cout << "" << std::endl;
    std::cout << "Speed: " << speed <<std::endl;
    std::cout << "Acceleration: " << acceleration <<std::endl;
    std::cout << "Current Points are entered: " << std::endl;
    for(auto &tmp : *points){
        tmp.output();
    }
}



Trajectory* Spline::calculateSpline() {
    out();
    Trajectory* trajectory = new Trajectory();
    std::vector<double> distance_i;
    std::vector<double> boundary_i;
    double di =0, tmp=0;                                                                                                                             // contains distances between point n and n-1
    double time_for_accel = this->acceleration/this->speed;
    double d_for_accel = 0.5 * this->acceleration*(time_for_accel*time_for_accel);
    timesteps = ConfigProvider::getInstance().getsteps_per_second();
    // initialize vector to store the chosen configurations at every timepoint


    // write start configuration to a new configuration data type
    Configuration* start_config = new Configuration({this->start_config[0], this->start_config[1], this->start_config[2],
                                                     this->start_config[3], this->start_config[4], this->start_config[5]});

    // add first config to config_Vec
   // config_vec.push_back(start_config);

    //------------------add this from robot class ----------------------------------------------------------------------
    double a_max = ConfigProvider::getInstance().getMax_accel();
    //setting max_velocity values for robot joints in °/s
    Vector<double, 6> joint_max_vel;
    joint_max_vel[0] = ConfigProvider::getInstance().getJoint1_max_vel();
    joint_max_vel[1] = ConfigProvider::getInstance().getJoint2_max_vel();
    joint_max_vel[2] = ConfigProvider::getInstance().getJoint3_max_vel();
    joint_max_vel[3] = ConfigProvider::getInstance().getJoint4_max_vel();
    joint_max_vel[4] = ConfigProvider::getInstance().getJoint5_max_vel();
    joint_max_vel[5] = ConfigProvider::getInstance().getJoint6_max_vel();

    for(int i =0; i<3 ; i++){
       // std::cout << "Start at i: "<< this->start_position[i] << std::endl;
       // std::cout << "points at i: "<< this->points->at(0)[i] << std::endl;
        tmp += (this->start_position[i] - this->points->at(0)[i])*(this->start_position[i] - this->points->at(0)[i]);
    }
    di = sqrt(tmp);
    std::cout << "Distances for first segment: " << di << std::endl;
    distance_i.push_back(di);

    for (unsigned int i = 0; i < this->points->size()-1; ++i) {
        tmp = 0;
        for(int j =0; j<3 ; j++){
            tmp += (this->points->at(i)[j] - this->points->at(i+1)[j])*(this->points->at(i)[j] - this->points->at(i+1)[j]);
        }
        di = sqrt(tmp);
        std::cout << "Distances for segment i: " << i << " : "<< di << std::endl;
        distance_i.push_back(di);
    }

    // First Derivative Heuristic
    std::cout << "" << std::endl;
    std::cout << "First Derivative Heuristic: " << std::endl;
    // for first point
    // Determine direction of trajectory from first to second point (straight line) and Scale with 0.5
    Vector<double, 3> p_first_direction_vec;
    std::cout << "number of points = " << num_points << std::endl;

    for (int i = 0; i < 3; ++i) {
        p_first_direction_vec[i] = 0.5 * (points->at(0)[i] - start_position[i]);
    }

    std::cout << "Spline: Direction of StartPoint: " << p_first_direction_vec[0] << ", " << p_first_direction_vec[1] << ", " << p_first_direction_vec[2] << std::endl;

    // for last point
    // Determine direction of trajectory from last to previous point (straight line)
    Vector<double, 3> p_last_direction_vec;
//    // check if we only have 2 points entered. Special Case!!
    if(num_points == 1){
        for (int i = 0; i < 3; ++i) {
            p_last_direction_vec[i] = 0.5 * (points->at(num_points-1)[i] - start_position[i]);
        }
        std::cout << "Spline: Direction of Last Point (Special Case): " << p_last_direction_vec[0]
        << ", " << p_last_direction_vec[1] << ", " << p_last_direction_vec[2] << std::endl;
    }else {             // check if we have more than 2 points. Normal Case!!!
        for (int i = 0; i < 3; ++i) {
            p_last_direction_vec[i] = 0.5 * (points->at(num_points-1)[i] - points->at(num_points - 2)[i]);
        }
        std::cout << "Spline: Direction of Last Point (Normal Case): " << p_last_direction_vec[0]
                  << ", " << p_last_direction_vec[1] << ", " << p_last_direction_vec[2] << std::endl;
  }

    // inner Waypoints
    std::cout << "" << std::endl;
    std::cout << "Spline: Inner Waypoints: " << std::endl;
    std::vector<Vector<double, 3>> inner_waypoint_dir_vec;
    Vector<double, 3> temporary_vec;
    if(num_points > 1) {
        for (int i = 0; i < num_points - 1; ++i) {
            for (int j = 0; j < 3; ++j) {
                temporary_vec[j] = 0.5 * (points->at(i + 1)[j] - points->at(i)[j]);
            }
            inner_waypoint_dir_vec.push_back(temporary_vec);
            std::cout << "Spline: Waypoint " << i + 1 << ": ";
            inner_waypoint_dir_vec.at(i).output();
        }
    }

    //calculate angle between vectors at the waypoints------------------------------------------------------------------
    std::vector<double> angles;
    for (int i = 0; i < num_points-1; ++i) {
        if(i == 0){
            Vector<double ,3> p_first_direction_temp = p_first_direction_vec * -1.0;
            angles.push_back( M_PII * 0.5 - 0.5 * acos( inner_waypoint_dir_vec.at(i).dot_product(p_first_direction_temp )
                                            / (p_first_direction_vec.length() * inner_waypoint_dir_vec.at(i).length()) ) );
            std::cout << "First angle: " << angles.at(0) << ". " << std::endl;
        }else{
            Vector<double, 3> inner_waypoint_dir_temp = inner_waypoint_dir_vec.at(i-1) * -1.0;
            double dot = inner_waypoint_dir_vec.at(i).dot_product(inner_waypoint_dir_temp);
            double len = (inner_waypoint_dir_vec.at(i).length() * inner_waypoint_dir_temp.length());
            double dot_len = dot/len;
            //catch rounding error
            if(dot_len > 1){
                dot_len = 1;
            }else if(dot_len < -1){
                dot_len = -1;
            }
            angles.push_back(M_PII*0.5 - (acos(dot_len) * 0.5));

            std::cout << "Next angle: " << angles.at(i) << ". " << std::endl;
        }
    }
    //with the angle we get calculate the tangent vectors at the inner_waypoints----------------------------------------
    std::vector<Vector<double, 3>> tangents;
    //add tangent of start point (has to be 0 though, because there is no velocity at this point yet)
    tangents.push_back(0.0 * p_first_direction_vec);
    Vector<double, 3> tangent_temp;
    tangents.reserve(inner_waypoint_dir_vec.size());
    for (unsigned int i = 0; i < inner_waypoint_dir_vec.size(); ++i) {
        //tangent_temp is our tangent, however the length is not 0.5 of the distance between the points
        tangent_temp = (inner_waypoint_dir_vec.at(i) * (1/cos(angles.at(i))));
        //wanted length is equal to the length of the inner_waypoints_dir_vec
        double wanted_length = inner_waypoint_dir_vec.at(i).length();
        double adjust_length = wanted_length/tangent_temp.length();
        //push final tangent with adjusted length to the vector
        tangents.push_back(tangent_temp * adjust_length);
    }
    //add tangent of endpoint (has to be 0 though, because there is no velocity anymore at this point)
    tangents.push_back(0.0 * p_last_direction_vec);

    // calculate inner waypoints of each segment-------------------------------------------------------------------------

    std::vector<Vector<double, 3>> total_point_vec;
    Vector<double, 3> point1_temp;
    Vector<double, 3> point2_temp;
    Vector<double, 3> point3_temp;
    Vector<double, 3> point4_temp;

    std::vector<Vector<double, 3>> a_vec;
    for (int i = 0; i < num_points; ++i) {
        //determine second derivatives (weight)

        if(i == 0){
            double alpha = distance_i.at(i+1)/(distance_i.at(i)+distance_i.at(i+1));
            double beta = distance_i.at(i)/(distance_i.at(i) + distance_i.at(i+1));

            Vector<double, 3> alpha_s = (6.0 * start_position);
            alpha_s = (2.0 * tangents.at(i)) + alpha_s;
            alpha_s = (4.0 * tangents.at(i+1)) + alpha_s;
            alpha_s = - 6.0 * points->at(i) + alpha_s;
            alpha_s = alpha_s * alpha;
            Vector<double, 3> alpha_e = -6.0 * points->at(i);
            alpha_e = -4.0 * tangents.at(i+1) + alpha_e;
            alpha_e = -2.0 * tangents.at(i+2) + alpha_e;
            alpha_e = 6.0 * points->at(i+1) + alpha_e;
            alpha_e = alpha_e * beta;
            Vector<double, 3> a_m = alpha_s + alpha_e;

            // determine second derivative for the first point (first segment)
            Vector<double, 3> a_first_segment = 6.0 * start_position;
            a_first_segment = 2.0 * tangents.at(i) + a_first_segment;
            a_first_segment = 4.0 * tangents.at(i + 1) + a_first_segment;
            a_first_segment = -6.0 * points->at(i) + a_first_segment;

            //set a of the first point to 0
            a_first_segment = 0.0 * a_first_segment;

            // determine second derivative for the last point (last segment)
            Vector<double, 3> a_last_segment = -6.0 * points->at(num_points - 2);
            a_last_segment = -4.0 * tangents.at(num_points - 1) + a_last_segment;
            a_last_segment = -2.0 * tangents.at(num_points) + a_last_segment;
            a_last_segment = 6.0 * points->at(num_points - 1) + a_last_segment;

            //set a of the last point to 0
            a_last_segment = 0.0 * a_last_segment;

            //push second derivatives into a vector
            a_vec.push_back(a_first_segment);
            a_vec.push_back(a_last_segment);
            a_vec.push_back(a_m);
            // point1 and 2 are equal to the start position (a & v = 0)
            point1_temp = start_position;
            point2_temp = start_position;
            point4_temp =  (- 0.2 * tangents.at(1)) + points->at(i);
            point3_temp = (0.05 * a_m);
            point3_temp = (2.0 * point4_temp) + point3_temp;
            point3_temp = -points->at(i) + point3_temp;

            total_point_vec.push_back(start_position);
            total_point_vec.push_back(point1_temp);
            total_point_vec.push_back(point2_temp);
            total_point_vec.push_back(point3_temp);
            total_point_vec.push_back(point4_temp);
            total_point_vec.push_back(points->at(i));
        }else if(i == num_points-1){                            //Last segment
            point1_temp = 0.2 * tangents.at(i) + points->at(i - 1);
            point2_temp = 0.05 * a_vec.at(i+1);
            point2_temp = 2.0 * point1_temp + point2_temp;
            point2_temp = - points->at(i - 1) + point2_temp;

            //point 3 and 4 are equal to the final point (a and v are 0 here)
            point3_temp = points->at(i);
            point4_temp =  points->at(i);

            total_point_vec.push_back(point1_temp);
            total_point_vec.push_back(point2_temp);
            total_point_vec.push_back(point3_temp);
            total_point_vec.push_back(point4_temp);
            total_point_vec.push_back(points->at(i));
        }else{                  //all the middle segments
            //need to add a_e of this segment to a_vec
            double alpha = distance_i.at(i+1)/(distance_i.at(i)+distance_i.at(i+1));
            double beta = distance_i.at(i)/(distance_i.at(i) + distance_i.at(i+1));

            Vector<double, 3> alpha_s = (6.0 * points->at(i-1));
            alpha_s = (2.0 * tangents.at(i)) + alpha_s;
            alpha_s = (4.0 * tangents.at(i+1)) + alpha_s;
            alpha_s = - 6.0 * points->at(i) + alpha_s;
            alpha_s = alpha_s * alpha;
            Vector<double, 3> alpha_e = -6.0 * points->at(i);
            alpha_e = -4.0 * tangents.at(i+1) + alpha_e;
            alpha_e = -2.0 * tangents.at(i+2) + alpha_e;
            alpha_e = 6.0 * points->at(i+1) + alpha_e;
            alpha_e = alpha_e * beta;
            Vector<double, 3> a_endpoint_middle_segment = alpha_s + alpha_e;
            a_vec.push_back(a_endpoint_middle_segment);

            point1_temp = 0.2 * tangents.at(i) + points->at(i-1);
            point2_temp = 0.05 * a_vec.at(i+1);
            point2_temp = 2.0 * point1_temp + point2_temp;
            point2_temp = -points->at(i - 1) + point2_temp;
            point4_temp =  (- 0.2 * tangents.at(i + 1)) + points->at(i);
            point3_temp = 0.05 * a_endpoint_middle_segment;
            point3_temp = 2.0 * point4_temp + point3_temp;
            point3_temp = -points->at(i) + point3_temp;

            total_point_vec.push_back(point1_temp);
            total_point_vec.push_back(point2_temp);
            total_point_vec.push_back(point3_temp);
            total_point_vec.push_back(point4_temp);
            total_point_vec.push_back(points->at(i));
        }

    }
    
    // determine points along the splinepath (for each segment)
    std::vector<Vector<double, 3>> spline_points;
    // vector to store time needed for every segment
    std::vector<double> time_vec;
    // bool to see if last segment is to short for decelerating
    bool last_to_short;
    // double t = double(j)/timesteps;
    double t = 0;
    double speed_temp = 0;
    // add distances of the segments together
    double moved_distance = 0;
    // add passed time of the segments together
    double passed_t = 0;
    // vector to store number of points for each segment
    std::vector<double> num_pts_segment;

    for (int i = 0; i < num_points; ++i) {
        int k = i * 5;

            // first segment
            // time when end effector reaches maximum velocity
            double t_temp = speed/acceleration;
            if(i == 0){
                // time needed for end effector to reach last point of first segment
                t = sqrt(2 * (distance_i.at(i) / acceleration));
                // check how long we need for the last segment
                double t_last_segment = sqrt(2 * (distance_i.at(distance_i.size()-1) / acceleration));
                if (t_temp < t_last_segment){
                    last_to_short = false;
                }else{
                    last_to_short = true;
                    // idea: set maximum velocity if its to short, so it is not to short anymore
                    speed = acceleration * t_last_segment;
                    t_temp = speed/acceleration;
                }
                if (t > t_temp){        // end effector reaches maximum velocity before reaching end point
                    double distance_for_accel = 0.5 * acceleration * t_temp * t_temp;
                    double time_with_constant_vel = (distance_i.at(i) - distance_for_accel) / speed;
                    t = t_temp + time_with_constant_vel;
                    speed_temp = speed;
                }else{                  // end effector does not reach max velocity
                    // time needed for end effector to reach last point of first segment
                    t = sqrt(2 * (distance_i.at(i) / acceleration));
                    // calculate speed at the end point
                    speed_temp = acceleration * t;
                }
                moved_distance = distance_i.at(i);
                passed_t = t;
                time_vec.push_back(t);
            }

            // other segment
            if( i > 0 && i < num_points-1){
                if (speed_temp == speed){                                                   // we have maximum velocity already
                    // time needed with constant velocity
                    t = distance_i.at(i) / speed;
                }else{                                                                      // we do not have maximum velocity yet
                    // time left for getting to max velocity
                    double t_left = t_temp - t;
                    // distance that the end effector moves when reaching maximum velocity (already has start velocity)
                    double distance_for_accel = 0.5 * acceleration * t_left * t_left + speed_temp * t_left;

                    // check if the distance of the segment is smaller than the distance moved by the robot to reach maximum velocity
                    if (distance_i.at(i) <= distance_for_accel){
                        // still did not reach maximum velocity. so we calculate the time when we reach the endpoint
                        // for this we calculate the time to reach the total distance traveled and subtract the already passed time
                        t = sqrt(2 * ( moved_distance + distance_i.at(i)) / acceleration) - passed_t;
                        // calculate speed that we have at the endpoint
                        speed_temp = acceleration * t + speed_temp;

                    }else{
                        // calculate distance left after reaching max velocity
                        double distance_for_const_vel = distance_i.at(i) - distance_for_accel;
                        // time we travel with constant velocity to reach end point
                        double time_with_constant_vel = distance_for_const_vel / speed;
                        // add times together to get time we need to reach end point
                        t = t_temp + time_with_constant_vel - passed_t;
                        // set speed at endpoint to max speed
                        speed_temp = speed;
                    }
                }
                passed_t = passed_t + t;
                moved_distance = moved_distance + distance_i.at(i);
                time_vec.push_back(t);
            }

            // last segment
            if (i == num_points-1){
                if (last_to_short){
                    // last segment is too short for decelerating from max speed to 0. Therefore we reduced the max speed to the maximum speed possible
                    t = sqrt(2 * (distance_i.at(i) / acceleration));
                    speed_temp = 0;
                }else{              // last segment is not too short for decelerating
                    // time needed for decelerating
                    double distance_decel =  0.5 * acceleration * t_temp * t_temp;
                    double distance_with_constant_vel = distance_i.at(i) - distance_decel;
                    double t_constant_vel = distance_with_constant_vel / speed_temp;
                    t = t_constant_vel + t_temp;
                    speed_temp = 0;
                }
                passed_t = passed_t + t;
                moved_distance = moved_distance + distance_i.at(i);
                time_vec.push_back(t);

            }
            // create points for each segment and push to spline_points
            double stepsize = 0;
            // add number of points for this segment to a vector
            num_pts_segment.push_back(round(t*timesteps + 1));
            for (int l = 0; l <= (t*timesteps); ++l) {
                stepsize += 1/(t*timesteps);
                spline_points.push_back(quintic_bezier_function(total_point_vec.at(k), total_point_vec.at(k+1), total_point_vec.at(k+2),
                                                                total_point_vec.at(k+3),total_point_vec.at(k+4), total_point_vec.at(k+5), stepsize));
            }

    }

    for(unsigned int i = 0; i < spline_points.size(); ++i) {
        std::cout << "Spline Point " << i << ": ";
        spline_points.at(i).output();
    }

    // get Configurations for every point in spline_points
    InvKinematics invKin;
    std::vector<Configuration*>* temp_configs = new vector<Configuration*>();

    //write initial velocities and accelerations into vectors (all are 0)
    vector<double> vel_accel_temp;
    vel_accel_temp.reserve(6);
    for (int j = 0; j < 6; ++j) {
        vel_accel_temp.push_back(0);
    }
    joint_velocities_vec.push_back(vel_accel_temp);

    for (unsigned int i = 0; i < spline_points.size(); ++i) {
        temp_configs = invKin.get_inv_kinematics(new SixDPos(spline_points.at(i)[0], spline_points.at(i)[1], spline_points.at(i)[2],
                                                             start_orientation[0] , start_orientation[1], start_orientation[2]));

        if (i == 0){
            // for the beginning we have a special case, because the difference between start cfg and first calculated is too large
            // we will end up with too high velocities and risk overshooting the next points
            std::vector<double> distances;
            double distance = 10000;
            int best_config = -1;
            for (unsigned int j = 0; j < temp_configs->size(); ++j) {

                if(calc_config_difference(start_config, temp_configs->at(j)) < distance){
                    // check if new config at new timepoint exceeds our limits of v & a
                        distance = calc_config_difference(start_config, temp_configs->at(j));
                        best_config = j;
                        std::cout << "distance of config " << j+1 << " is smaller than previous one" << std::endl;
                    }else{
                        std::cout << "distance of config " << j+1 << " is smaller than previous one" << std::endl;
                        std::cout << "However the new config exceeds robot limits" << std::endl;
                    }
                }
            config_vec.push_back(temp_configs->at(best_config));
            }

        if(temp_configs->size() != 0 && i != 0){
            std::vector<double> distances;
            double distance = 10000;
            int best_config = -1;
            for (unsigned int j = 0; j < temp_configs->size(); ++j) {

                if(calc_config_difference(config_vec.at(config_vec.size()-1), temp_configs->at(j)) < distance){
                    // check if new config at new timepoint exceeds our limits of v & a
                    if( checkconfiglimits(config_vec.at(config_vec.size()-1), temp_configs->at(j),
                                          &joint_velocities_vec.at(i-1), a_max, joint_max_vel, timesteps) ){
                        distance = calc_config_difference(config_vec.at(config_vec.size()-1), temp_configs->at(j));
                        best_config = j;
                        std::cout << "distance of config " << j+1 << " is smaller than previous one" << std::endl;
                    }else{
                        std::cout << "distance of config " << j+1 << " is smaller than previous one" << std::endl;
                        std::cout << "However the new config exceeds robot limits" << std::endl;
                    }
                }
            }
            // check if there was a config that was not exceeding our robot limits
            if(best_config != -1){
                std::cout << "For the given SixDPos at time t " << i/timesteps << " there are possible configurations" << std::endl;
                // add best config to configuration vector
                std::cout << "The best configuration at t: " << i/timesteps << " is: " << best_config << std::endl;

                config_vec.push_back(temp_configs->at(best_config));
            }else{
                std::cout << "WARNING. At t: " << i/timesteps << std::endl;
                perror("There was no possible configuration in robot limits!!!! ");
                // insert function that determines new configurations between two configurations that are not reachable yet (because of limits)
                // first determine closest configuration (this time without checking the limits)
                for (unsigned int j = 0; j < temp_configs->size(); ++j) {
                    if (calc_config_difference(config_vec.at(config_vec.size() - 1), temp_configs->at(j)) < distance) {
                        distance = calc_config_difference(config_vec.at(config_vec.size()-1), temp_configs->at(j));
                        best_config = j;
                    }
                }
                // here we check the limits of the chosen configuration (middle config gets pushed into config_vec inside the function itself)
                add_middle_cfg(config_vec.at(config_vec.size()-1), temp_configs->at(best_config),
                                                        &joint_velocities_vec.at(i-1), a_max, joint_max_vel, timesteps);


            }
        }else{
            std::cout << "ERROR: For the given SixDPos at time t " << i/timesteps << " are no possible configurations available!!!" << std::endl;
        }
    }
    if (config_switch_indent != -1){
        std::vector<Configuration*>::iterator it = config_vec.begin();
        while (it != config_vec.end()){
            if (*it != config_vec.at(config_switch_indent)){
                it++;
            }else{
                config_vec.erase(it, config_vec.end());
                break;
            }
        }
    }

    trajectory->set_trajectory(config_vec);


    return trajectory;
}

double Spline::calc_num(double stemp, int n) {
    double result = (1-stemp);
    if(n == 0)
        return 1;
    else
        for(int i = 1; i<n; i++)
        {
            result *= (1-stemp);
        }
    return result;
}

Vector<double, 3> Spline::quintic_bezier_function(Vector<double, 3> point0, Vector<double, 3> point1, Vector<double, 3> point2, Vector<double, 3> point3,
                                          Vector<double, 3> point4, Vector<double, 3> point5, double t){
   // check if t is > 1 due to rounding issues
    if (t > 1){
        t = 1;
    }
    Vector<double, 3> s_t, tmp0,tmp1,tmp2,tmp3,tmp4,tmp5;
    double coeff0 = calc_num(t,5);
    double coeff1 = 5*calc_num(t,4) * t;
    double coeff2 = 10*calc_num(t,3) * (t) * (t);
    double coeff3 = 10*calc_num(t,2) * t * t * t;
    double coeff4 = 5*calc_num(t,1)*t * t * t * t;
    double coeff5 = t * t * t * t * t;

    tmp0 = coeff0*point0;
    tmp1 = coeff1*point1;
    tmp2 = coeff2*point2;
    tmp3 = coeff3*point3;
    tmp4 = coeff4*point4;
    tmp5 = coeff5*point5;

    s_t = tmp0 + tmp1 + tmp2 + tmp3 + tmp4 + tmp5;
    return s_t;
}

double Spline::calc_config_difference(Configuration* config1, Configuration* config2){

    double distance_joint1 = config2->get_configuration().operator[](0) - config1->get_configuration().operator[](0);
    double distance_joint2 = config2->get_configuration().operator[](1) - config1->get_configuration().operator[](1);
    double distance_joint3 = config2->get_configuration().operator[](2) - config1->get_configuration().operator[](2);
    double distance_joint4 = config2->get_configuration().operator[](3) - config1->get_configuration().operator[](3);
    double distance_joint5 = config2->get_configuration().operator[](4) - config1->get_configuration().operator[](4);
    double distance_joint6 = config2->get_configuration().operator[](5) - config1->get_configuration().operator[](5);
    std::vector<double> distances_joints_vec;
    distances_joints_vec.push_back(distance_joint1);
    distances_joints_vec.push_back(distance_joint2);
    distances_joints_vec.push_back(distance_joint3);
    distances_joints_vec.push_back(distance_joint4);
    distances_joints_vec.push_back(distance_joint5);
    distances_joints_vec.push_back(distance_joint6);

    // check if one of the distances is too large (changing from forward to backward case for example)
    for (int i = 0; i < 6; ++i) {
        if(distances_joints_vec.at(i) > (config_switch_limit * M_PII/180) ){
            config_switch_indent = config_vec.size()-1;
        }
    }

    return abs(distance_joint1) + abs(distance_joint2) + abs(distance_joint3) + abs(distance_joint4) + abs(distance_joint5) + abs(distance_joint6);
}

bool Spline::checkconfiglimits(Configuration* config1, Configuration* config2,
                            std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps) {
    double distance_joint1 = config2->get_configuration().operator[](0) * 180/M_PII - config1->get_configuration().operator[](0) * 180/M_PII;
    double distance_joint2 = config2->get_configuration().operator[](1) * 180/M_PII - config1->get_configuration().operator[](1) * 180/M_PII;
    double distance_joint3 = config2->get_configuration().operator[](2) * 180/M_PII - config1->get_configuration().operator[](2) * 180/M_PII;
    double distance_joint4 = config2->get_configuration().operator[](3) * 180/M_PII - config1->get_configuration().operator[](3) * 180/M_PII;
    double distance_joint5 = config2->get_configuration().operator[](4) * 180/M_PII - config1->get_configuration().operator[](4) * 180/M_PII;
    double distance_joint6 = config2->get_configuration().operator[](5) * 180/M_PII - config1->get_configuration().operator[](5) * 180/M_PII;

    //calculate needed acceleration of each joint to arrive at next config in time interval given by timesteps
    double t = 1 / timesteps;
    double joint1_curr_a = 2 * (distance_joint1 - (velocities_vec->at(0) * t)) / (t * t);
    double joint2_curr_a = 2 * (distance_joint2 - (velocities_vec->at(1) * t)) / (t * t);
    double joint3_curr_a = 2 * (distance_joint3 - (velocities_vec->at(2) * t)) / (t * t);
    double test = (distance_joint4 - (velocities_vec->at(3) * t));
    double test2 = 2 * test;
    double test3 = test2/(t * t);
    double joint4_curr_a = 2 * (distance_joint4 - (velocities_vec->at(3) * t)) / (t * t);
    double joint5_curr_a = 2 * (distance_joint5 - (velocities_vec->at(4) * t)) / (t * t);
    double joint6_curr_a = 2 * (distance_joint6 - (velocities_vec->at(5) * t)) / (t * t);
    //calculate velocity of each joint after the given time interval
    std::vector<double> velocity_curr_vec;
    velocity_curr_vec.push_back( velocities_vec->at(0) + joint1_curr_a * t );
    velocity_curr_vec.push_back( velocities_vec->at(1) + joint2_curr_a * t );
    velocity_curr_vec.push_back( velocities_vec->at(2) + joint3_curr_a * t );
    velocity_curr_vec.push_back( velocities_vec->at(3) + joint4_curr_a * t );
    velocity_curr_vec.push_back( velocities_vec->at(4) + joint5_curr_a * t );
    velocity_curr_vec.push_back( velocities_vec->at(5) + joint6_curr_a * t );

    // check if accelerations are over the limit
    if(abs(joint1_curr_a) > a_max || abs(joint2_curr_a) > a_max || abs(joint3_curr_a) > a_max || abs(joint4_curr_a) > a_max ||
       abs(joint5_curr_a) > a_max || abs(joint6_curr_a) > a_max ){
        return false;
    }
    // check if velocities are over the limit
    for (int i = 0; i < 6; ++i) {
        if (abs(velocities_vec->at(i)) > joint_v_max[i]){
            return false;
        }
    }
    //push current velocity in global vector so we store all velocities at the points
    joint_velocities_vec.push_back(velocity_curr_vec);

    return true;
}
void Spline::add_middle_cfg2(Configuration* config1, Configuration* config2,
                            std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps) {
    std::vector<double> distances_joints_vec;
    double distance_joint1 = config2->get_configuration().operator[](0) * 180/M_PII - config1->get_configuration().operator[](0) * 180/M_PII;
    double distance_joint2 = config2->get_configuration().operator[](1) * 180/M_PII - config1->get_configuration().operator[](1) * 180/M_PII;
    double distance_joint3 = config2->get_configuration().operator[](2) * 180/M_PII - config1->get_configuration().operator[](2) * 180/M_PII;
    double distance_joint4 = config2->get_configuration().operator[](3) * 180/M_PII - config1->get_configuration().operator[](3) * 180/M_PII;
    double distance_joint5 = config2->get_configuration().operator[](4) * 180/M_PII - config1->get_configuration().operator[](4) * 180/M_PII;
    double distance_joint6 = config2->get_configuration().operator[](5) * 180/M_PII - config1->get_configuration().operator[](5) * 180/M_PII;
    distances_joints_vec.push_back(distance_joint1);
    distances_joints_vec.push_back(distance_joint2);
    distances_joints_vec.push_back(distance_joint3);
    distances_joints_vec.push_back(distance_joint4);
    distances_joints_vec.push_back(distance_joint5);
    distances_joints_vec.push_back(distance_joint6);

    // create new config in the middle of the 2 configs
    double new_cfg_j1 = (config1->get_configuration().operator[](0) * 180/M_PII + distance_joint1 * 0.5) * M_PII/180;
    double new_cfg_j2 = (config1->get_configuration().operator[](1) * 180/M_PII + distance_joint2 * 0.5) * M_PII/180;
    double new_cfg_j3 = (config1->get_configuration().operator[](2) * 180/M_PII + distance_joint3 * 0.5) * M_PII/180;
    double new_cfg_j4 = (config1->get_configuration().operator[](3) * 180/M_PII + distance_joint4 * 0.5) * M_PII/180;
    double new_cfg_j5 = (config1->get_configuration().operator[](4) * 180/M_PII + distance_joint5 * 0.5) * M_PII/180;
    double new_cfg_j6 = (config1->get_configuration().operator[](5) * 180/M_PII + distance_joint6 * 0.5) * M_PII/180;

    Configuration *cfg_middle = new Configuration(
            {new_cfg_j1, new_cfg_j2, new_cfg_j3, new_cfg_j4, new_cfg_j5, new_cfg_j6});

   // checkconfiglimits(config1, cfg_middle);

}

void Spline::add_middle_cfg(Configuration* config1, Configuration* config2,
                               std::vector<double> *velocities_vec, double a_max, Vector<double, 6> joint_v_max, double timesteps) {
    std::vector<double> distances_joints_vec;
    double distance_joint1 = config2->get_configuration().operator[](0) * 180/M_PII - config1->get_configuration().operator[](0) * 180/M_PII;
    double distance_joint2 = config2->get_configuration().operator[](1) * 180/M_PII - config1->get_configuration().operator[](1) * 180/M_PII;
    double distance_joint3 = config2->get_configuration().operator[](2) * 180/M_PII - config1->get_configuration().operator[](2) * 180/M_PII;
    double distance_joint4 = config2->get_configuration().operator[](3) * 180/M_PII - config1->get_configuration().operator[](3) * 180/M_PII;
    double distance_joint5 = config2->get_configuration().operator[](4) * 180/M_PII - config1->get_configuration().operator[](4) * 180/M_PII;
    double distance_joint6 = config2->get_configuration().operator[](5) * 180/M_PII - config1->get_configuration().operator[](5) * 180/M_PII;
    distances_joints_vec.push_back(distance_joint1);
    distances_joints_vec.push_back(distance_joint2);
    distances_joints_vec.push_back(distance_joint3);
    distances_joints_vec.push_back(distance_joint4);
    distances_joints_vec.push_back(distance_joint5);
    distances_joints_vec.push_back(distance_joint6);

    // check if one of the distances is too large (changing from forward to backward case for example)
//    for (int i = 0; i < 6; ++i) {
//        if(distances_joints_vec.at(i) > config_switch_limit){
//            config_switch_indent = config_vec.size()-1;
//        }
//    }

    //calculate needed acceleration of each joint to arrive at next config in time interval given by timesteps
    double t = 1 / timesteps;
    std::vector<double> joint_curr_a_vec;
    joint_curr_a_vec.push_back(2 * (distance_joint1 - (velocities_vec->at(0) * t)) / (t * t));
    joint_curr_a_vec.push_back(2 * (distance_joint2 - (velocities_vec->at(1) * t)) / (t * t));
    joint_curr_a_vec.push_back(2 * (distance_joint3 - (velocities_vec->at(2) * t)) / (t * t));
    joint_curr_a_vec.push_back(2 * (distance_joint4 - (velocities_vec->at(3) * t)) / (t * t));
    joint_curr_a_vec.push_back(2 * (distance_joint5 - (velocities_vec->at(4) * t)) / (t * t));
    joint_curr_a_vec.push_back(2 * (distance_joint6 - (velocities_vec->at(5) * t)) / (t * t));


    // check if accelerations are over the limit and determine joint with highest acceleration
    double highest_a = 0;
    int limit_joint = -1;
    for (unsigned int i = 0; i < joint_curr_a_vec.size(); ++i) {
        if (abs(joint_curr_a_vec.at(i)) > a_max){
            std::cout << "Joint " << i << " needed acceleration is too high (" << abs(joint_curr_a_vec.at(i)) << " > " << a_max << ")." << std::endl;
            // find out the highest acceleration that exceeds the limit
            if (abs(joint_curr_a_vec.at(i)) > highest_a){
                highest_a = abs(joint_curr_a_vec.at(i));
                limit_joint = i;
            }
        }
    }

    // joint with highest acceleration is the limiting factor, so we determine the difference in angle (distance) this joint can reach in the given time interval
    // we need to check the direction of this joint (sign of distance)
    double possible_distance = 0;
    if (distances_joints_vec.at(limit_joint) < 0 && joint_curr_a_vec.at(limit_joint) < 0){
        // distance is negative, acceleration is negative
        possible_distance = -0.5 * a_max * t * t + velocities_vec->at(limit_joint) * t;
    } else if(distances_joints_vec.at(limit_joint) > 0 && joint_curr_a_vec.at(limit_joint) > 0){
        // acceleration and distance are both positive
        possible_distance = 0.5 * a_max * t * t + velocities_vec->at(limit_joint) * t;
    } else if( distances_joints_vec.at(limit_joint) > 0 && joint_curr_a_vec.at(limit_joint) < 0 ){
        //distance is positive, acceleration is negative
        possible_distance = -0.5 * a_max * t * t + velocities_vec->at(limit_joint) * t;
    } else if( distances_joints_vec.at(limit_joint) < 0 && joint_curr_a_vec.at(limit_joint) > 0 ){
        // distance is negative, acceleration is positive
        possible_distance = 0.5 * a_max * t * t + velocities_vec->at(limit_joint) * t;
    }

    // determine the factor of the possible distance with the distance before when we exceed the limit
    double factor = abs(possible_distance / distances_joints_vec.at(limit_joint));
    // this factor will be used to determine new configuration that will be between the two configs from the beginning
    std::vector<double> adj_distances_joints_vec;
    for (unsigned int i = 0; i < distances_joints_vec.size(); ++i) {
        adj_distances_joints_vec.push_back(distances_joints_vec.at(i) * factor);
    }
    // determine the new middle config with the help of the adj_distances
    double new_cfg_j1 = ( config1->get_configuration().operator[](0) * 180/M_PII + adj_distances_joints_vec.at(0) ) * M_PII/180;
    double new_cfg_j2 = ( config1->get_configuration().operator[](1) * 180/M_PII + adj_distances_joints_vec.at(1) ) * M_PII/180;
    double new_cfg_j3 = ( config1->get_configuration().operator[](2) * 180/M_PII + adj_distances_joints_vec.at(2) ) * M_PII/180;
    double new_cfg_j4 = ( config1->get_configuration().operator[](3) * 180/M_PII + adj_distances_joints_vec.at(3) ) * M_PII/180;
    double new_cfg_j5 = ( config1->get_configuration().operator[](4) * 180/M_PII + adj_distances_joints_vec.at(4) ) * M_PII/180;
    double new_cfg_j6 = ( config1->get_configuration().operator[](5) * 180/M_PII + adj_distances_joints_vec.at(5) ) * M_PII/180;


    //calculate velocity of each joint after the given time interval
    std::vector<double> velocity_curr_vec;
    velocity_curr_vec.push_back( velocities_vec->at(0) + joint_curr_a_vec.at(0) * factor * t );
    velocity_curr_vec.push_back( velocities_vec->at(1) + joint_curr_a_vec.at(1) * factor * t );
    velocity_curr_vec.push_back( velocities_vec->at(2) + joint_curr_a_vec.at(2) * factor * t );
    velocity_curr_vec.push_back( velocities_vec->at(3) + joint_curr_a_vec.at(3) * factor * t );
    velocity_curr_vec.push_back( velocities_vec->at(4) + joint_curr_a_vec.at(4) * factor * t );
    velocity_curr_vec.push_back( velocities_vec->at(5) + joint_curr_a_vec.at(5) * factor * t );

    // check if velocities are over the limit
    bool vel_too_high = false;
    for (int i = 0; i < 6; ++i) {
        if (abs(velocities_vec->at(i)) > joint_v_max[i]){
            vel_too_high = true;
            std::cout << "Velocities for the new adjusted config are too high" << std::endl;
        }
    }
    if(!vel_too_high) {
        //push current velocity in global vector so we store all velocities at the points
        joint_velocities_vec.push_back(velocity_curr_vec);

        // create new config
        Configuration *cfg1dot5 = new Configuration(
                {new_cfg_j1, new_cfg_j2, new_cfg_j3, new_cfg_j4, new_cfg_j5, new_cfg_j6});
        //now we have an additional configuration between the original 2 configurations
        // push config into global vector storing all configs
        config_vec.push_back(cfg1dot5);
        temp_vec.push_back(cfg1dot5);

        // we can use this config to check if between config 2 and 3 the limits are exceeded. (we only checked between config 1 and 2 now)
        bool cfg1dot5_to_cfg2_in_limits = checkconfiglimits(cfg1dot5, config2,
                                                            &velocity_curr_vec, a_max, joint_v_max, timesteps);
        if (!cfg1dot5_to_cfg2_in_limits) {
            // we need additional points between cfg1dot5 and cfg2
            add_middle_cfg(cfg1dot5, config2,
                           &velocity_curr_vec, a_max, joint_v_max, timesteps);
        }else{
            // push config2 into global vector
            config_vec.push_back(config2);
        }
    }else{
        // add function
        std::cout << "We reached our limits of velocity" << std::endl;
    }

    config_vec.push_back(config2);

}

