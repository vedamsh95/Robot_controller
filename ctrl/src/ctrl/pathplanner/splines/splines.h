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

#ifdef PLOT
namespace plt = matplotlibcpp;
#endif

/*
* This class implements the geometrical computation of cubic or qunitic bezier splines to get a smooth curve between the given points.
* Furthermore the distance between spline points are determined to fulfill the specified velocity and acceleration
*/
class Splines
{
public:

	Splines();
	~Splines();
	/**
     * Get quintic or cubic bezier spline by vector of base/knot points.
	 * The control points between these points are calculated to get polynomial function representing a smooth curve 
	 * The orientation while driving down the spline stays the same
     *
     * @param  {@ref SixDPos} Position and Orientation (constant) of the knot points  {@ref Configuration} start Config
	 * @param  {@ref Configuration} start Configuration of the robot at the first point
	 * @param  maximal velocity of the robot
	 * @param  maximal acceleration of the robot
	 * @out	   loop Points that are not part of the spline but have to be visited by the robot because of limits of the joints.
	 *              Use a nullptr in case this functionality is not needed.
	 * @param  elongation Factor determines length of tangents of quintic splines (default: half of distance between adjacent points)
	 * @param  spline type 
	 *		   0 : cubic bezier spline (default)
	 *		   1 : quintic bezier spline
     * @return geometrical spline trajectory in consideration of velocity and acceleration of the robot (distance between adjacent points points)
     *         In case the spline got aborted, the trajectory contains a zero configuration at the end!
	 */
	Trajectory *getSpline(vector<SixDPos*> &_points, Configuration * start, double _velocity, double _acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double _elong = 0.5, int spline_type = 0);
	
	/**
	* Struct to store 3D Coordinate and handle arithemtic operations
	*/
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
	/**
     * Plot 3D Spline, velocity profil and t Values of Spline 
     *
     * @param  Position of calculated spline points
	 * @param  t Values of each spline seqment 
	 */
	void plotSpline(vector<Splines::Position> &_points, vector<double> _tValues);
	/**
     * First Derivative Heuristic Quintic 
	 * Angle of tangent at each inner waypoint W_i is set to be perpendicular 
	 * to the angular bisector of the line seqments L_i and L_i-1
     * Default magnitude half of Euclidean distance from W_i to closest waypoint
	 * T_i = e_i * min{||L_i-1||, ||L_i||} * (L_i-1 / ||L_i-1|| + L_i/ ||L_i||)
     * with L_i = W_i+1 - W_i
	 * @param  Position knot/waypoints W of spline
	 * @param  constant elongation factor e_i 
	 */
	std::vector<Position> getTangentsQuintic(vector<Splines::Position> &_points, double _elong);
	
	/**
     * Control points of cubic bezier splines
	 * @param  waypoints of Spline
	 * @out first control points of spline seqments
	 * @out second control points of spline seqments
	 */
	void getCubicBezierControlPoints(vector<Splines::Position> &_points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	

	/**
     * Second Derivative Heuristic Qunitic
	 * weighted average of second derivatives of cubic spline 
	 * A_i = Q''_i-1(1) = Q''_i(0) = ||L_i||/(||L_i-1|| + ||L_i||) * C''_i-1(1) + ||L_i-1||/(||L_i-1|| + ||L_i||) * C''_i(0)
	 * A = alpha * S''_AB(1) + beta * S''_BC(0) = alpha *(6A + 2t_A + 4t_B - 6B) + beta * (-6B - 4t_B - 2t_C + 6C)
	 * A, B, C  neighbouring waypoints
	 * t_A, t_B, t_C cubic tangents at waypoints
	 * alpha = d_BC/(d_AB + d_BC)	beta = d_AB/(d_AB + d_BC)	d_AB: Euclidean Distance AB
	 * @param Position of waypoints
	 * @param tangents at waypoints
	 * @return accelerations at waypoints
	 */
	std::vector<Position> getAccelerationsQuintic(vector<Splines::Position> &_points, std::vector<Splines::Position> _tangents);
	
	/**
     * L2 Norm of 3D Point 
	 * Used for Magnitude of P and for Euclidean Distance of P1- P2 
     * @param 3D Position 
     * @return L2 Norm
     */
	double L2Norm(Position _P);

	/**
	 * Calculate Qunitic Bezier Spline
	 * Get discrete point of i-th spline seqment at specific t-Value [0,1] 
	 * 
	 * @param t percentage of spline seqment
	 * @param i number of spline seqment
	 * @param waypoints
	 * @param tangents at waypoints
	 * @param accelerations at waypoints
	 * 
	 * @return Position of spline point
	 */
	Position getQuinticBezier(double _t, int _i, std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations);
	/**
	 * Calculate Cubic Bezier Spline
	 * Get discrete point of i-th spline seqment at specific t-Value [0,1] 
	 * 
	 * @param t percentage of spline seqment
	 * @param i number of spline seqment
	 * @param waypoints
	 * @param first controlpoints of each seqment 
	 * @param second controlpoints of each seqment 
	 * 
	 * @return Position of spline point
	 */
	Position getCubicBezier(double _t, int _i, std::vector<Position> _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints);
	
	/**
	 * Arc Length Parametrization
	 * Sample Quinitc or Cubic Spline 
	 * each Spline seqment i is sampeled by vector<int> samples[i] points
	 * 
	 * @param required parameters to get Cubic or Qunitic Spline (Waypoints, Controllpoints or Second and First Derivative  
	 * @out _sampleDist Distance at each sampled point -> last entry is full distance of spline 
	 */
	void sampleDistancesCubic(std::vector<Position> _points, std::vector<Position> _firstControlPoints, std::vector<Position> _secondControlPoints, vector<double>* _sampleDist);
	void sampleDistancesQuintic(std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations, vector<double>* _sampleDist);
	
	/**
	 * get discrete t Values for each robot.time_interval [0,1] (percentage of spline) depending on velocity profile
	 * for further understanding look at max_vel_trajectory.h and trapeeziodal_trajectory.h
	 * @param distance of spline
	 * @param max velocity of robot
	 * @param max acceleration of robot
	 * @return t Values 
	 */
	vector<double> getTValues(double _distance, double _velocity, double _acceleration);

	Trajectory *trajectory;
	Robot *robot;
	IVMovement* ivMovement;
	FwKinematics* fwK;
	bool plot;
	vector<int> samples;
	double sampleDistance;

};