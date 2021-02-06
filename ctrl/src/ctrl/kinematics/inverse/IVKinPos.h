#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <array>
#include <vector>
#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>
#include <Robot.h>
/*This class implements the inverse kinematic of the first three joints of the robot*/
class IVKinPos
{
public:
	IVKinPos();
	~IVKinPos();
	/**
     * Get Inverse Kinematic of the first three joints
     *
     * @param  {@ref SixDPos} Position and Orientation 
     * @return Array with angles of first three joint in degree
	 */
	std::vector<std::array<double, 3>*>* get_IVKinPos(SixDPos* _pos);
	
	 /**
     * Get Wrist Center Point
     * static public function for handling the ellbow singularities
     * @param  {@ref SixDPos} Position and Orientation 
     * @return Array with coordinates of the wrist center point
	 */
	static std::array<double, 3> getWristCenterPoint(SixDPos* _pos);


private:
	/**
     * Struct to store 3D Coordinate and handle arithemtic operations
	 */
	struct Position
	{
		double x, y, z;
		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

		inline Position operator+(Position a) {
			return { a.x + x,a.y + y, a.z + z };
		}
	};
	double m;
	double n;
	double a;
	double o;
	double b;

	//distance from wrist center point to tool center point in [m]
	static constexpr double d_6 = 0.215;

	static constexpr double marginSingularity = 0.001;
	/**
	 * calculating all possible configuration of this robot by given angle and position of the wristPoint
	 *
	 * @param  angle of first joint _phi1
	 * @param  _P Position of the wrist point
	 * @return all possible configuration of the first three joints
	 */
	std::vector<std::array<double, 3>*>* calc_configurations(double _phi1, Position _P);
	/**
	* calculating ellbow up and down configurations of forward solution
	*
	* @param  _d1 distance from wrist Point to Base in z plane
	* @param  coordinates of center wrist point
	* @return angles of ellbow upward und downward solutions of second and third joint
		phi2_forward_upward,
		phi2_forward_downward,
		phi3_forward_upward,
		phi3_forward_downward
	*/
	std::array<double, 4> forward_calc(double _d1, Position _wristPoint);
	/**
	 * calculating ellbow up and down configurations of backward solution
	 *
	 * @param  _d1 distance from wrist Point to Base in z plane
	 * @param  coordinates of center wrist point
	 * @return angles of ellbow upward und downward solutions of second and third joint
			phi2_backward_upward,
			phi2_backward_downward,
			phi3_backward_upward,
			phi3_backward_downward
	 */
	std::array<double, 4> backward_calc(double _d1, Position _wristPoint);
	/**
     * check limits of calculated angles and assemble all possible solutions
	 * 
     * @param  angle of first joint _phi1 
	 * @param  _solution all configurations (ellbow up and down) of second and third joint
     * @out _ans every for this robot possible configuration of second and third joint at orientation _phi1 
	 */
	void checkLimits(double _phi1, std::array<double, 4>* _solution, std::vector<std::array<double, 3>*>* _ans);
	/**
     * checking if offset of base column from the Z axis "m" is greater or lesser then _d1
	 * 
     * @param  _d1 distance from wrist Point to Base in z plane 
     * @return boolean if  d1 greater/equal (true) or less then m (false); no offset (true)
	 */
	bool d1Condition(double _d1);
	/**
	* angle conversion from degree to radian
	* @param  angle in degree
	* @return angle in radian
	*/
	double deg2Rad(double _deg);
	/**
	 * angle conversion from radian to degree
	 * @param  angle in radian
	 * @return angle in degree
	 */
	double rad2Deg(double _rad);

	Robot *robot;
};

