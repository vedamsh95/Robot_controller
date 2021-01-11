#include "IVKinPos.h"
#include<iostream>


IVKinPos::IVKinPos()
{
}

IVKinPos::IVKinPos(double _x, double _y, double _z)
{
	P.x = _x;
	P.y = _y;
	P.z = _z;
}

std::vector<std::array<double, 3>*>* IVKinPos::get_IVKinPos()
{
	double phi1;
	//Standard 1
	if ((P.x > 0 && P.y > 0))
	{
		phi1 = rad2Deg(-atan(P.y / P.x));
		return standardCase(phi1, P);
		
	}
	if (P.x > 0 && P.y < 0)
	{
		phi1 = rad2Deg(atan(P.y / P.x));
		return standardCase(phi1, P);
	}
	//Standard 2
	if (P.x < 0 && P.y < 0)
	{
		phi1 = rad2Deg(M_PI - atan(P.y / P.x));
		return standardCase(phi1, P);
	}
	// Standard 3
	if (P.x < 0 && P.y > 0)
	{
		phi1 = rad2Deg(M_PI - atan(P.y / -P.x));
		return standardCase(phi1, P);
	}
	// Special Case 1
	if (P.x == 0 && P.y > 0)
	{
		return specialcase1(P);
	}
	// Special Case 2
	if (P.x == 0 && P.y < 0)
	{
		return specialcase2(P);
	}

}


IVKinPos::~IVKinPos()
{	
}

double IVKinPos::deg2Rad(double _deg)
{
	return _deg/ (180.0 / M_PI);
}

double IVKinPos::rad2Deg(double _rad)
{
	return _rad * (180.0 / M_PI);
}

std::array<double, 3>* IVKinPos::checkLimits(double _phi1, std::array<double, 5>* _solution)
{
	static std::array<double, 3> ans;
	if (-185.0 < _phi1 && _phi1 < (185.0))
	{
		if ((-140.0) < _solution->at(0) && _solution->at(0) < (-5.0)) //
		{
			if ((-120.0) < _solution->at(2) && _solution->at(2) <(168))
			{
				ans = { _phi1,_solution->at(0),_solution->at(2) };
				return &ans;
			}
				
		}
		if ((-140.0) < _solution->at(1) && _solution->at(1) < (-5.0)) //
		{
			if ((-120.0) < _solution->at(3) && _solution->at(3) < (168))
			{
				ans = { _phi1,_solution->at(1),_solution->at(3) };
				return &ans;
			}
	
			if ((-120.0) < _solution->at(4) && _solution->at(4) < (168))
			{
				ans = { _phi1,_solution->at(1),_solution->at(4) };
				return &ans;
			}
		}
	
	}
	return nullptr;
}

std::vector<std::array<double, 3>*>* IVKinPos::standardCase(double _phi1, Position _P)
{
	double d1 = std::sqrt(P.x + P.x + P.y * P.y);
	double px_dash, py_dash;
	std::array<double, 3>* sol;
	std::array<double, 5>* solution;
	static std::vector<std::array<double, 3>*> ans;
	if ((-185.0) < _phi1 && _phi1 < (-175.0))
	{
		return checkphi1_case1(_phi1, _P);
	}
	if((175.0) < _phi1 &&  _phi1 < (185.0))
	{
		return checkphi1_case2(_phi1, _P);
	}
	if ((-175.0) <= _phi1 && _phi1 <= (175.0))
	{
		if (d1 > m && P.z > n)//?
		{
			std::cout << "forward case " << std::endl;
			px_dash = d1 - m;
			py_dash = P.z - n;
			solution = forward_calc(_phi1, px_dash, py_dash);
			std::array<double, 3>* sol = checkLimits(_phi1, solution);
			if (sol != nullptr)
				ans.push_back(sol);
			sol = otherCase1(_phi1, d1, P);
			if (sol != nullptr)
				ans.push_back(sol);
			sol = otherCase2(_phi1, d1, P);
			if (sol != nullptr)
				ans.push_back(sol);
		}
		if (d1 < n)
		{
			std::cout << "backward case " << std::endl;
			px_dash = m - d1;
			py_dash = P.z;
			solution = backward_calc(_phi1, px_dash, py_dash);

			std::array<double, 3>* sol = checkLimits(_phi1, solution);
			if (sol != nullptr)
				ans.push_back(sol);
			sol = otherCase1(_phi1, d1, P);
			if (sol != nullptr)
				ans.push_back(sol);
			sol = otherCase2(_phi1, d1, P);
			if (sol != nullptr)
				ans.push_back(sol);
		}
		
	}

	return &ans;
}

std::array<double, 3>* IVKinPos::otherCase1(double _phi1, double _d1, Position _P)
{
	double px_dash = _d1 - m;//? +
	double py_dash = _P.z - n;
	
	double phi1_new1 = _phi1 + 180.0;
	std::array<double, 5>* solution = backward_calc(phi1_new1, px_dash, py_dash);
	return checkLimits(phi1_new1, solution);
}

std::array<double, 3>* IVKinPos::otherCase2(double _phi1, double _d1, Position _P)
{
	double px_dash = _d1 - m;
	double py_dash = _P.z - n;

	double phi1_new2 = _phi1 - 180.0;
	std::array<double, 5>* solution = backward_calc(phi1_new2, px_dash, py_dash);
	return checkLimits(phi1_new2, solution);
}

std::vector<std::array<double, 3>*>* IVKinPos::checkphi1_case1(double _phi1, Position _P)
{
	double d1 = sqrt(P.x*P.x + P.y * P.y);
	double px_dash, py_dash, phi1_new;
	std::array<double, 3>* sol;
	static std::vector<std::array<double, 3>*> ans;
	
	if (d1 > m && P.z > n)
	{
		px_dash = d1 - m;
		py_dash = P.z - n;
		
		sol = checkLimits(_phi1,forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);
		
		phi1_new =_phi1 + 360.0;
		sol = checkLimits(phi1_new, forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = phi1_new + 180.0;//?
		px_dash = d1 + m;
		sol = checkLimits(phi1_new, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		
	}
	if (d1 < m)
	{
		px_dash = m - d1;
		py_dash = P.z - n;

		sol = checkLimits(_phi1, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = _phi1 + 360.0;
		sol = checkLimits(phi1_new, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = phi1_new +180.0;//?
		px_dash = d1 + m;
		sol = checkLimits(phi1_new, forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		
	}

	return &ans;
}

std::vector<std::array<double, 3>*>* IVKinPos::checkphi1_case2(double _phi1, Position _P)
{

	double d1 = sqrt(P.x*P.x + P.y * P.y);
	double px_dash, py_dash, phi1_new;
	std::array<double, 3>* sol;
	static std::vector<std::array<double, 3>*> ans;

	if (d1 > m && P.z > n)
	{
		px_dash = d1 - m;
		py_dash = P.z - n;

		sol = checkLimits(_phi1, forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = _phi1 - 360.0;
		sol = checkLimits(phi1_new, forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = phi1_new - 180;//?
		px_dash = d1 + m;
		sol = checkLimits(phi1_new, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);
	}
	if (d1 < m)
	{
		px_dash = m - d1;
		py_dash = P.z - n;

		sol = checkLimits(_phi1, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = _phi1 - 360.0;
		sol = checkLimits(phi1_new, backward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1_new = phi1_new - 180.0;//?
		px_dash = d1 + m;
		sol = checkLimits(phi1_new, forward_calc(_phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);
	}

	return &ans;
}

std::vector<std::array<double, 3>*>* IVKinPos::specialcase1(Position _P)
{
	double d1, phi1, px_dash, py_dash;
	std::array<double, 3>* sol;
	static std::vector<std::array<double, 3>*> ans;
	if (P.y > m)
	{
		 d1= P.y;
	     phi1 = -90.0;
		 px_dash = P.y - m;
		 py_dash = P.z - n;

		 sol = checkLimits(phi1,forward_calc(phi1, px_dash, py_dash));
		 if (sol != nullptr)
			 ans.push_back(sol);

		 phi1 = 90.0;
		 px_dash = P.y + m;

		 sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		 if (sol != nullptr)
			 ans.push_back(sol);
		
		 return &ans;
	}
	if (P.y < m)
	{
		d1 = P.y;
		phi1 = -90.0;
		px_dash = m - P.y;
		py_dash = P.z - n;

		sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1 = 90.0;
		px_dash = P.y + m;

		sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		return &ans;
	}
	return nullptr;
}

std::vector<std::array<double, 3>*>* IVKinPos::specialcase2(Position _P)
{
	double d1, phi1, px_dash, py_dash;
	std::array<double, 3>* sol;
	std::vector<std::array<double, 3>*> ans;
	if (P.y > m)
	{
		d1 = P.y;
		phi1 = 90.0;
		px_dash = P.y - m;
		py_dash = P.z - n;

		sol = checkLimits(phi1, forward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1 = -90.0;
		px_dash = P.y + m;

		sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		return &ans;
	}
	if (P.y < m)
	{
		d1 = P.y;
		phi1 = 90.0;
		px_dash = m - P.y;
		py_dash = P.z - n;

		sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		phi1 = -90.0;
		px_dash = P.y + m;

		sol = checkLimits(phi1, backward_calc(phi1, px_dash, py_dash));
		if (sol != nullptr)
			ans.push_back(sol);

		return &ans;

	}
	return nullptr;
}




std::array<double, 5>* IVKinPos::forward_calc(double _phi1, double _px_dash, double _py_dash)
{
	double d3 = sqrt(_px_dash* _px_dash + _py_dash * _py_dash);
	double d2 = sqrt(o*o + b * b);
	double beta = rad2Deg(acos(((d3*d3) - (a*a) - (d2*d2)) / (-2 * a*d2)));
	double alpha1 = rad2Deg(asin(sin(deg2Rad(beta)) * (d2 / d3)));
	double alpha2 = rad2Deg(asin(_py_dash / d3));
	double phi2_forward_upward = -1.0 * (alpha2 + alpha1);
	double phi2_forward_downward = -1.0 * (alpha2 - alpha1);
	double phi3_forward_upward = 360.0 - beta - rad2Deg(asin(b / d2)) - 90.0;
	double phi3_forward_downward = -	1.0 *(90.0 - (360.0 - rad2Deg(asin(b / d2)) - (2 * 180.0 - beta)));
	double phi3_forward_upward_1 = -1.0 * (90.0 + rad2Deg(asin(b / d2))- beta);
	static std::array<double, 5> ans = 
	{
		phi2_forward_upward,
		phi2_forward_downward,
		phi3_forward_upward,
		phi3_forward_downward,
		phi3_forward_upward_1
	};
	return &ans;
}

std::array<double, 5>* IVKinPos::backward_calc(double _phi1, double _px_dash, double _py_dash)
{
	double d3 = sqrt(_px_dash* _px_dash + _py_dash * _py_dash);
	double d2 = sqrt(o*o + b * b);
	double beta = rad2Deg(acos(((d3*d3) - (a*a) - (d2*d2)) / (-2 * a*d2)));
	double alpha1 = rad2Deg(asin(sin(deg2Rad(beta)) * (d2 / d3)));
	double alpha2 = rad2Deg((asin(_py_dash / d3)));
	double phi2_backward_upward = (alpha2 + alpha1) - 180.0; 
	double phi2_backward_downward = (alpha2 - alpha1) - 180.0;
	
	double phi3_backward_upward = -1.0 * (90.0 - 1.0 * (beta - rad2Deg(asin(b / d2))));
	double phi3_backward_downward = (360.0 - beta - rad2Deg(asin(b / d2)) - 90.0);
	double phi3_backward_downward_1 = ((3 / 2) *180.0) - beta - rad2Deg(asin(b / d2));
	static std::array<double, 5> ans =
	{
		phi2_backward_upward,
		phi2_backward_downward,
		phi3_backward_upward,
		phi3_backward_downward,
		phi3_backward_downward_1
	};
	return &ans;
}


