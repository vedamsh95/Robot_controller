#pragma once 
#include<SixDPos.h>
#include <Trajectory.h>
#include <Robot.h>
#include<math.h>
#include <array>
#include<vector>
#include<iostream>

//#include "inverse_kinematics.h"
class Splines
{
public:

	//              Trajectory* trajectory = ctrl.move_robot_spline(points, vel, acc);
	Splines();
	~Splines();
	Trajectory *getSpline(vector<SixDPos*> &_points, double _velocity, double _acceleration);
	struct Position
	{
		double x, y, z;


		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

		inline Position operator+(Position a) {
			return { a.x + x,a.y + y, a.z + z };
		}
		inline Position operator-(Position a) {
			return { x - a.x, y- a.y, z - a.z};
		}
		inline Position operator/(double a) {
			return { x/a,y/a, z/a};
		}
		inline Position operator*(double a) {
			return { x * a,y * a, z * a };
		}

	};
private:

	std::vector<Position> getTagents(vector<Splines::Position> &_points);
	std::vector<Position> getAccelerations(vector<Splines::Position> &_points, std::vector<Splines::Position> _tangents);
	double L2Norm(Position _p1);
	Position getQuinticBezier(double _t, int _i, std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations);
	Trajectory *trajectory;
	Robot *robot;



	
};