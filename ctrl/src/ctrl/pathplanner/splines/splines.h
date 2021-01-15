#pragma once 
#include<SixDPos.h>
#include <Trajectory.h>
#include<math.h>
#include <array>
#include<vector>

class Splines
{
public:
	//              Trajectory* trajectory = ctrl.move_robot_spline(points, vel, acc);
	Splines();
	~Splines();
	Trajectory *getQuinticBezier(vector<SixDPos*> &points, double _velocity, double _acceleration);
private:
	struct Position
	{
		double x, y, z;


		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

		inline Position operator+(Position a) {
			return { a.x + x,a.y + y, a.z + z };
		}
	};
};