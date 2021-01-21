#pragma once 
#include<SixDPos.h>
#include <Trajectory.h>
#include <Robot.h>
#include<math.h>
#include <array>
#include<vector>
#include<iostream>

#include"../kinematics/inverse/IVMovement.h"
#include"../kinematics/direct/fw_kinematics.h"
#include "../ptp/Single_trajectory.h"
#include "../ptp/Max_vel_trajectory.h"
#include "../ptp/Trapezoidal_trajectory.h"
//#include <functional>
#include "../../matplotlib-cpp-master/matplotlibcpp.h"
namespace plt = matplotlibcpp;
class Splines
{
public:

	//              Trajectory* trajectory = ctrl.move_robot_spline(points, vel, acc);
	Splines();
	~Splines();
	Trajectory *getSpline(vector<SixDPos*> &_points, Configuration * start, double _velocity, double _acceleration);
	struct Position
	{
		double x, y, z;


		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
		Position(double _v) : x(_v), y(_v), z(_v) {};
		Position(SixDPos* sixDPos) : x(sixDPos->get_X()), y(sixDPos->get_Y()), z(sixDPos->get_Z()) {};

		inline Position operator+(Position a) {
			return { a.x + x,a.y + y, a.z + z };
		}
		inline Position operator-(Position a) {
			return { x - a.x, y- a.y, z - a.z};
		}
		inline Position operator/(double a) {
			return { x/a,y/a, z/a};
		}
		inline Position operator/(Position a) {
			return { x / a.x,y / a.y, z / a.z };
		}
		inline Position operator*(double a) {
			return { x * a,y * a, z * a };
		}
		inline Position operator*(Position a) {
			return { x * a.x,y * a.y, z * a.z };
		}

	};
private:
	void plotSpline(vector<Splines::Position> &_points);
	std::vector<Position> getTagents(vector<Splines::Position> &_points);
	void getCubicBezierControlPoints(vector<Splines::Position> &_points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	
	std::vector<Position> getAccelerations(vector<Splines::Position> &_points, std::vector<Splines::Position> _tangents);
	

	double L2Norm(Position _p1);
	Position getQuinticBezier(double _t, int _i, std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations);
	Position getCubicBezier(double _t, int _i, std::vector<Position> _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	
	//int operation2(int x, int y, std::function<int(int, int)> function) { return function(x, y); }
	//int operation(int x, int y, int(*function)(int, int)) { return function(x, y); }
	//int sub(int x, int y) { return x - y; }
	double getSplineLength(std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations);
	//double getSplineLength(std::vector<Position> _points, std::vector<Position> _parameter1, std::vector<Position> _parameter2, std::function<Position(double _t, int _i,std::vector<Position>, std::vector<Position>, std::vector<Position>)> function);
	double getSplineLengthCubic(std::vector<Position> _points, std::vector<Position> _firstControlPoints, std::vector<Position> _secondControlPoints);
	vector<double> getTValues(double _distance, double _velocity, double _acceleration);
	Trajectory *trajectory;
	Robot *robot;

	IVMovement* ivMovement;
	FwKinematics* fwK;
	bool plot;



	
};