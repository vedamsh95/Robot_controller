//
// Created by Timo on 07.01.2021.
//

#include "spline.h"
#include "Vector.h"
#include "math.h"
#include <iostream>
#include "Trajectory.h"
#include "ConfigProvider.h"


Spline::Spline(Vector<double, 3> start_point, Vector<double, 3> start_orientation,
               std::vector<Vector<double, 3>> *points, double speed, double acceleration) {

    this->start_position[0] = start_point[0];
    this->start_position[1] = start_point[1];
    this->start_position[2] = start_point[2];

    this->start_orientation[0] = start_orientation[0];
    this->start_orientation[1] = start_orientation[1];
    this->start_orientation[2] = start_orientation[2];

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
    Trajectory* trajectory = new Trajectory();
    std::vector<double> distance_i;
    std::vector<double> boundary_i;
    double di =0, tmp=0;                                                                                                                             // contains distances between point n and n-1
    double time_for_accel = this->acceleration/this->speed;
    double d_for_accel = 0.5 * this->acceleration*(time_for_accel*time_for_accel);
    double M_PII = 3.141592654;


    for(int i =0; i<3 ; i++){
       // std::cout << "Start at i: "<< this->start_position[i] << std::endl;
       // std::cout << "points at i: "<< this->points->at(0)[i] << std::endl;
        tmp += (this->start_position[i] - this->points->at(0)[i])*(this->start_position[i] - this->points->at(0)[i]);
    }
    di = sqrt(tmp);
    std::cout << "Distances for first segment: " << di << std::endl;
    distance_i.push_back(di);

    for (int i = 0; i < this->points->size()-1; ++i) {
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
    //add tangent of start point
    tangents.push_back(p_first_direction_vec);
    Vector<double, 3> tangent_temp;
    tangents.reserve(inner_waypoint_dir_vec.size());
    for (int i = 0; i < inner_waypoint_dir_vec.size(); ++i) {
        //tangent_temp is our tangent, however the length is not 0.5 of the distance between the points
        tangent_temp = (inner_waypoint_dir_vec.at(i) * (1/cos(angles.at(i))));
        //wanted length is equal to the length of the inner_waypoints_dir_vec
        double wanted_length = inner_waypoint_dir_vec.at(i).length();
        double adjust_length = wanted_length/tangent_temp.length();
        //push final tangent with adjusted length to the vector
        tangents.push_back(tangent_temp * adjust_length);
    }
    //add tangent of endpoint
    tangents.push_back(p_last_direction_vec);

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

            // determine second derivative for the last point (last segment)
            Vector<double, 3> a_last_segment = -6.0 * points->at(num_points - 2);
            a_last_segment = -4.0 * tangents.at(num_points - 1) + a_last_segment;
            a_last_segment = -2.0 * tangents.at(num_points) + a_last_segment;
            a_last_segment = 6.0 * points->at(num_points - 1) + a_last_segment;

            //push second derivatives into a vector
            a_vec.push_back(a_first_segment);
            a_vec.push_back(a_last_segment);
            a_vec.push_back(a_m);

            point1_temp = 0.2 * tangents.at(i) + start_position;
            point2_temp = 0.05 * a_first_segment;
            point2_temp = 2.0 * point1_temp + point2_temp;
            point2_temp = -start_position + point2_temp;
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
            point4_temp =  (- 0.2 * tangents.at(i + 1)) + points->at(i);
            point3_temp = 0.05 * a_vec.at(1);
            point3_temp = 2.0 * point4_temp + point3_temp;
            point3_temp = - points->at(i) + point3_temp;

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









    return nullptr;
}



/*
Spline::Spline(double _t, double _ts, double _te, double _as, double _ae, Vector<double,3> start, Vector<double,3> end) {
    this->current_timestamp = 0;
    this->t =_t;
    this->t_s = _ts;
    this->t_e = _te;
    this->a_s = _as;
    this->a_e = _ae;

    this->p0 = start;
    this->p5 = end;

//-------------calculations -----------------//

    this->p1 = ((1/5)*_ts) + start;

    this->p2[0] = (1/20) * _as + 2.0*this->p1[0] -this->p0[0];
    this->p2[1] = (1/20) * _as + 2.0*this->p1[1] -this->p0[1];
    this->p2[2] = (1/20) * _as + 2.0*this->p1[2] -this->p0[2];

    this->p4[0] = end[0] -(1/5)*_te;
    this->p4[1] = end[1] -(1/5)*_te;
    this->p4[2] = end[2] -(1/5)*_te;

    this->p3[0] = (1/20) * _ae + 2*this->p4[0] - end[0];
    this->p3[1] = (1/20) * _ae + 2*this->p4[1] - end[1];
    this->p3[2] = (1/20) * _ae + 2*this->p4[2] - end[2];



}

double Spline::get_current_timestamp() const {
    return this->current_timestamp;
}

void Spline::recalc_timestamp() {
    this->current_timestamp += this->t;

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

std::vector<Vector<double, 3>> Spline::calc_spline() {

    std::vector<Vector<double,3>> spline;
    Vector<double, 3> res, tmp0,tmp1,tmp2,tmp3,tmp4,tmp5;

    while(this->get_current_timestamp() <=1)
    {
        double coeff0 = calc_num(this->current_timestamp,5);
        double coeff1 = 5*calc_num(this->current_timestamp,4) * this->current_timestamp;
        double coeff2 = 10*calc_num(this->current_timestamp,3) * (this->current_timestamp) * (this->current_timestamp);
        double coeff3 = 10*calc_num(this->current_timestamp,2)*this->current_timestamp * this->current_timestamp*this->current_timestamp;
        double coeff4 = 5*calc_num(this->current_timestamp,1)*this->current_timestamp * this->current_timestamp*this->current_timestamp*this->current_timestamp;
        double coeff5 = this->current_timestamp * this->current_timestamp*this->current_timestamp*this->current_timestamp*this->current_timestamp;

        tmp0 = coeff0*this->p0;
        tmp1 = coeff1*this->p1;
        tmp2 = coeff2*this->p2;
        tmp3 = coeff3*this->p3;
        tmp4 = coeff4*this->p4;
        tmp5 = coeff5*this->p5;

        res = tmp0 + tmp1 + tmp2 + tmp3 + tmp4 + tmp5;
        std::cout << this->get_current_timestamp() << std::endl;
        this->recalc_timestamp();

        spline.push_back(res);
    }

    return spline ;
}




// -----------How to handle----------- //
/*
Vector<double, 3> start;
start[0] = 5.5;
start[1] = 7.9;
start[2] = 4.3;


Vector<double, 3> end;
end[0] = 16.9;
end[1] = 12.9;
end[2] = 13.9;

Vector<double, 3> test;

start.output();
end.output();
test = (2.0*start)+end;
test.output();

Spline sp(0.1,0.1,0.1,0.1,0.1,start,end);
std::cout << sp.get_current_timestamp() << std::endl;

std::vector<Vector<double,3>> result;

result = sp.calc_spline();

for(Vector<double,3> &i :  result){
i.output();
}

*/
