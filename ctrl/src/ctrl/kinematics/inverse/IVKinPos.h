#pragma once
#define _USE_MATH_DEFINES
#include<math.h>
#include <array>
#include<vector>
#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>
class IVKinPos
{
	/*
	Calculate the position of the wrist point
	*/

public:
	IVKinPos();
	~IVKinPos();

	std::vector<std::array<double, 3>*>* get_IVKinPos(SixDPos* _pos);

	struct Position
	{
		double x, y, z;
		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

		inline Position operator+(Position a) {
			return { a.x + x,a.y + y, a.z + z };
		}
	};
private:
	static constexpr double m = 0.330;
	static constexpr double n = 0.645;
	static constexpr double a = 1.150;
	static constexpr double o = 0.115;
	static constexpr double b = 1.220;
	static constexpr double d_6 = 0.215;

	static constexpr double marginSingularity = 0.001;

	
	double deg2Rad(double _deg);
	double rad2Deg(double _rad);
	void checkLimits(double _phi1, std::array<double, 4>* _solution, std::vector<std::array<double, 3>*>* _ans);

	std::vector<std::array<double, 3>*>* calc_configurations(double _phi1, Position _P);
	


	bool d1Condition(double _d1);
	std::array<double,4> forward_calc(double _d1, Position _wristPoint);
	std::array<double,4> backward_calc(double _d1, Position _wristPoint);
};

