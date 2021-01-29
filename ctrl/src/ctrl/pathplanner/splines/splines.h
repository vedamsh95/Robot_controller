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
#include "../../matplotlib-cpp-master/matplotlibcpp.h"
namespace plt = matplotlibcpp;
class Splines
{
public:

	Splines();
	~Splines();
	Trajectory *getSpline(vector<SixDPos*> &_points, Configuration * start, double _velocity, double _acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double _elong = 0.5, int spline_type = 0);
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
	void plotSpline(vector<Splines::Position> &_points, vector<double> _tValues);
	std::vector<Position> getTangentsQuintic(vector<Splines::Position> &_points, double _elong);
	void getCubicBezierControlPoints(vector<Splines::Position> &_points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	
	std::vector<Position> getAccelerationsQuintic(vector<Splines::Position> &_points, std::vector<Splines::Position> _tangents);
	

	double L2Norm(Position _p1);
	Position getQuinticBezier(double _t, int _i, std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations);
	Position getCubicBezier(double _t, int _i, std::vector<Position> _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	

	void sampleDistancesCubic(std::vector<Position> _points, std::vector<Position> _firstControlPoints, std::vector<Position> _secondControlPoints, vector<double>* _sampleDist);
	void sampleDistancesQuintic(std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations, vector<double>* _sampleDist);
	
	
	vector<double> getTValues(double _distance, double _velocity, double _acceleration);
	Trajectory *trajectory;
	Robot *robot;

	IVMovement* ivMovement;
	FwKinematics* fwK;
	bool plot;
	int sample;
	vector<int> samples;
	double sampleDistance;

};