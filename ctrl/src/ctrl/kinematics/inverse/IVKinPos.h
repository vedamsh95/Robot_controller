#pragma once
#define _USE_MATH_DEFINES
#include<math.h>
#include <array>
#include<vector>
class IVKinPos
{
	/*
	Calculate the position of the wrist point
	*/

public:
	IVKinPos();
	IVKinPos(double _x, double _y, double _z);

	std::vector<std::array<double, 3>*>* get_IVKinPos();

	~IVKinPos();
private:
	static constexpr double m = 0.330;
	static constexpr double n = 0.645;
	static constexpr double a = 1.150;
	static constexpr double o = 0.115;
	static constexpr double b = 1.220;


		
	struct Position
	{
		double x, y, z;
		Position() : x(0), y(0), z(0) {};
		Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
	};
	double deg2Rad(double _deg);
	double rad2Deg(double _rad);
	Position P = Position(); // wristPointCenter
	//double joints[3];
	std::array<double, 3>* checkLimits(double _phi1, std::array<double, 5>* _solution);
	std::vector<std::array<double, 3>*>* standardCase(double _phi1, Position _P);
	std::array<double, 3>* otherCase1(double _phi1, double _d1, Position _P);
	std::array<double, 3>* otherCase2(double _phi1, double _d1, Position _P);
	//lässt sich mit weniger code implementieren
	std::vector<std::array<double, 3>*>* checkphi1_case1(double _phi1, Position _P);
	std::vector<std::array<double, 3>*>* checkphi1_case2(double _phi1, Position _P);
	
	std::vector<std::array<double, 3>*>* specialcase1(Position _P);
	std::vector<std::array<double, 3>*>* specialcase2(Position _P);


	std::array<double,5>* forward_calc(double _phi1, double _px_dash, double _py_dash);
	std::array<double,5>* backward_calc(double _phi1, double _px_dash, double _py_dash);
};

