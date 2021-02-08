#include <cmath>
#include <cfloat>
#include <iostream>
#include "spline.h"
#include "../../kinematics/inverse/inverse_kinematics.h"

#define SECONDS_PER_STEP 0.05
#define PTP_DEBUG true

const double MAX_VELOCITY[6] = {120, 115, 120, 190, 180, 260}; // in °/s
const double MAX_ACCELERATION[6] = {300, 300, 300, 300, 300, 300}; // in °/s²

vector<double> *distances = new vector<double>();
vector<double> *boundaries = new vector<double>();
vector<SixDPos *> *path;

Trajectory *Spline::get_spline_trajectory(Configuration *start, vector<SixDPos *> _path) {
    //computation of a spline trajectory with the corresponding velocity profile
    Trajectory *trajectory = new Trajectory();

    vector<SixDPos *> solvedPath;
    path = &_path;

    // each 6 points are a path element - if there are not enough points, duplicate the last one
    // TODO: use t & d vectors for easier control (and actual C1/C2 continuity!)
    // TODO: better control point UI
    while ((path->size() - 1) % 5 != 0) path->push_back(path->back());

    distances->clear();
    distances->push_back(0);
    double distance = 0;
    for (int i = 0; i < path->size() - 1; i += 5) {
        double dist = 0;
        // Summing up all distances here to allow loops
        for (int offset = 0; offset < 5; offset++) {
            dist += sqrt(
                    pow(path->at(i + offset)->get_X() - path->at(i + offset + 1)->get_X(), 2) +
                    pow(path->at(i + offset)->get_Y() - path->at(i + offset + 1)->get_Y(), 2) +
                    pow(path->at(i + offset)->get_Z() - path->at(i + offset + 1)->get_Z(), 2)
            );
        }
        distance += dist;
        distances->push_back(dist);
    }
    boundaries->clear();
    boundaries->push_back(0);
    for (int i = 1; i < distances->size(); i++) {
        boundaries->push_back(boundaries->back() + distances->at(i) / distance);
    }

    double progress = 0; // time value from 0 to 1
    double speed = 0.01; // step resolution, can be made lower to make huge movements smoother
    while (progress < 1) {
        solvedPath.push_back(get_spline_at(progress));
        progress += speed;
    }
    solvedPath.push_back(get_spline_at(1));

    for (int i = 0; i < solvedPath.size(); i++) {
        cout << "Spline @ t = " << i * SECONDS_PER_STEP << " Seconds (Step #" << i << "):"
             << " X=" << solvedPath.at(i)->get_X()
             << " Y=" << solvedPath.at(i)->get_Y()
             << " Z=" << solvedPath.at(i)->get_Z()
             << " A=" << solvedPath.at(i)->get_A()
             << " B=" << solvedPath.at(i)->get_B()
             << " C=" << solvedPath.at(i)->get_C() << endl;
    }

    if (pathOnly) return nullptr;

    vector<Configuration *> config = {start};
    for (int i = 0; i < solvedPath.size(); i++) {
        // inverse kinematics
        // TODO: make this a function in inverse_kinematics.h
        InvKinematics invKinematics;
        vector<Configuration *> *new_cfg = invKinematics.get_inv_kinematics(solvedPath.at(i));
        Configuration *bestConfiguration = nullptr;
        double bestDistance = DBL_MAX;
        for (int i = 1; i < new_cfg->size(); i++) {
            double distance = 0;
            for (int j = 0; j < 6; j++) {
                distance += abs((*(*new_cfg)[i])[j] - (*solvedPath.at(i - 1))[j]) * MAX_VELOCITY[j];
            }
            if (distance < bestDistance) {
                bestDistance = distance;
                bestConfiguration = (*new_cfg)[i];
            }
        }
        // TODO: add potentially neccessary steps in-between to respect the robot's physical limits.
        config.push_back(bestConfiguration);
    }
    trajectory->set_trajectory(config);
    return trajectory;
}

SixDPos *Spline::get_spline_at(double t) {
    int segment = 1;
    while (segment < boundaries->size() - 1 && boundaries->at(segment) <= t) segment++;
    // we now got the right boundary, but we'll need the left one, so we'll subtract 1 again.
    segment--;

    // internal time for the polynomial piece
    t = (t - boundaries->at(segment)) / (
            (segment == boundaries->size() - 2 ? 1 : boundaries->at(segment + 1))
            - boundaries->at(segment));
    SixDPos *result = new SixDPos();
    vector<double> position;
    for (int i = 0; i < 6; i++) {
        double v = 0;
        // Quintic Bezier Spline (this is probably optimizable!)
        v += pow(1 - t, 5) * path->at(segment * 5)->operator[](i);
        v += 5 * pow(1 - t, 4) * t * path->at(segment * 5 + 1)->operator[](i);
        v += 10 * pow(1 - t, 3) * pow(t, 2) * path->at(segment * 5 + 2)->operator[](i);
        v += 10 * pow(1 - t, 2) * pow(t, 3) * path->at(segment * 5 + 3)->operator[](i);
        v += 5 * (1 - t) * pow(t, 4) * path->at(segment * 5 + 4)->operator[](i);
        v += pow(t, 5) * path->at(segment * 5 + 5)->operator[](i);
        position.push_back(v);
    }
    result->set_position(position);
    return result;
}