#include "IVKinPos.h"
#include<iostream>


IVKinPos::IVKinPos()
{
}
IVKinPos::~IVKinPos()
{
}


std::vector<std::array<double, 3>*>* IVKinPos::get_IVKinPos(SixDPos* _pos)
{
	double phi1;
	//Wrist Point
	//_pos = new SixDPos(0.0, 0.0, 1.5, 0.0, 1.0, 1.0);
	TMatrix m_transEndeffector2Wrist = TMatrix(_pos->get_C(), _pos->get_B(), _pos->get_A(), _pos->get_X(), _pos->get_Y(), _pos->get_Z());
	Position wristPoint;
	wristPoint.x = _pos->get_X() - d_6 * m_transEndeffector2Wrist.get(0, 2);
	wristPoint.y = _pos->get_Y() - d_6 * m_transEndeffector2Wrist.get(1, 2);
	wristPoint.z = _pos->get_Z() - d_6 * m_transEndeffector2Wrist.get(2, 2);

	std::cout << "wristPoint " << wristPoint.x << " " << wristPoint.y << " " << wristPoint.z << endl;
	double maximumDistance = sqrt(wristPoint.x  *wristPoint.x + wristPoint.y*wristPoint.y + wristPoint.z * wristPoint.z);

	if (wristPoint.x > -marginSingularity && wristPoint.x < marginSingularity &&
		wristPoint.y > -marginSingularity && wristPoint.y < marginSingularity)
	{
		phi1 = 90.0;
		std::cout << "Overhead Singularity" << endl;
	}
	else
		phi1 = rad2Deg(atan2(wristPoint.y, wristPoint.x));//vielleicht noch - zeichen, da clockwise vs counterclockwise
	
	std::cout << "phi1 " << phi1 << endl;
	return calc_configurations(phi1, wristPoint);
	////1 Quadrant
	//if ((wristPoint.x > 0 && wristPoint.y > 0))
	//{
	//	std::cout << "1 Quadrant " << std::endl;
	//	//eigentlich + phi1 aber in formeln -
	//	phi1 = rad2Deg(atan(wristPoint.y / wristPoint.x));
	//	return standardCase_phi1(phi1, wristPoint);
	//	
	//}
	//// 4. Quadrant
	//if (wristPoint.x > 0 && wristPoint.y < 0)
	//{
	//	std::cout << "4 Quadrant " << std::endl;
	//	phi1 = rad2Deg(atan(wristPoint.y / wristPoint.x));
	//	return standardCase_phi1(phi1, wristPoint);
	//}
	//// 3. Quadrant
	//if (wristPoint.x < 0 && wristPoint.y < 0)
	//{
	//	std::cout << "3 Quadrant " << std::endl;
	//	phi1 = rad2Deg(M_PI - atan(wristPoint.y / wristPoint.x));
	//	return standardCase_phi1(phi1, wristPoint);
	//}
	////4. Quadrant
	//if (wristPoint.x < 0 && wristPoint.y > 0)
	//{
	//	std::cout << "4 Quadrant " << std::endl;
	//	phi1 = rad2Deg(M_PI - atan(wristPoint.y / -wristPoint.x));
	//	return standardCase_phi1(phi1, wristPoint);
	//}
	// Special Case 1
	if (wristPoint.x == 0 && wristPoint.y > 0)
	{
		std::cout << "Special Case 1 " << std::endl;
		
	}
	// Special Case 2
	if (wristPoint.x == 0 && wristPoint.y < 0)
	{
		std::cout << "Special Case 2 " << std::endl;

	}

}


double IVKinPos::deg2Rad(double _deg)
{
	return _deg/ (180.0 / M_PI);
}

double IVKinPos::rad2Deg(double _rad)
{
	return _rad * (180.0 / M_PI);
}

void IVKinPos::checkLimits(double _phi1, std::array<double, 4>* _solution, std::vector<std::array<double, 3>*>* _ans)
{
	/*phi2_backward_upward,
		phi2_backward_downward,
		phi3_backward_upward,
		phi3_backward_downward,
		phi3_backward_downward_1*/
	//TODO Werte in Roboter Klasse nehmen
	std::cout << "checkLimits " << endl;
	std::array<double, 3>* config = new std::array<double, 3>();
	if (-185.0 < _phi1 && _phi1 < (185.0))
	{
		if ((-140.0) < _solution->at(0) && _solution->at(0) < (-5.0)) //
		{
			if ((-120.0) < _solution->at(2) && _solution->at(2) < (168))
			{
				std::cout << "Downwards" << endl;
				config = new std::array<double, 3>{ _phi1, _solution->at(0), _solution->at(2) };
				_ans->push_back(config);
			}

		}
		if ((-140.0) < _solution->at(1) && _solution->at(1) < (-5.0)) //
		{
			if ((-120.0) < _solution->at(3) && _solution->at(3) < (168))
			{
				std::cout << "Upwards" << endl;
				config = new std::array<double, 3>{ _phi1,_solution->at(1),_solution->at(3) };
				_ans->push_back(config);
			}
		}

	}
}




std::vector<std::array<double, 3>*>* IVKinPos::calc_configurations(double _phi1, Position _wristPoint)
{
	std::array<double, 4>* forwardSolutions;
	std::array<double, 4>* backwardSolutions;
	std::vector<std::array<double, 3>*>* sol;
	std::vector<std::array<double, 3>*>* ans = new std::vector<std::array<double, 3>*>();


	double d1 = std::sqrt(_wristPoint.x * _wristPoint.x + _wristPoint.y * _wristPoint.y);


	forwardSolutions = forward_calc(d1, _wristPoint);
	

	checkLimits(_phi1, forwardSolutions, ans);

	if ((-185.0) <= _phi1 && _phi1 <= (-175.0))
	{
		checkLimits(360.0 +_phi1, forwardSolutions, ans);
	}
	if ((185.0) >= _phi1 && _phi1 >= (175.0))
	{
		checkLimits(-(360.0- _phi1), forwardSolutions, ans);
	}



	backwardSolutions = backward_calc(d1, _wristPoint);
	

	if ((5) >= _phi1 && _phi1 >= (-5.0))
	{
		checkLimits(-(_phi1+180.0), backwardSolutions, ans);
		checkLimits(+(_phi1 + 180.0), backwardSolutions, ans);

	}
	else
	{
		double phi1_backward = rad2Deg(atan2(-_wristPoint.y, -_wristPoint.x));// oder if anweisungen welcher Quadrant
		checkLimits(phi1_backward, backwardSolutions, ans);
	}
	

	return ans;
}




bool IVKinPos::d1Condition(double _d1)
{
	return (_d1 >= m || _d1 == 0);
}

std::array<double, 4>* IVKinPos::forward_calc(double _d1, Position _wristPoint)
{
	std::cout << "forward_calc: " << endl;
	double px_dash, py_dash;
	if (d1Condition(_d1))
		px_dash = _d1 - m;
	else
		px_dash = m - _d1;

	py_dash = _wristPoint.z - n;

	double d2 = sqrt(o*o + b * b);
	double d3 = sqrt(px_dash* px_dash + py_dash * py_dash);

	double beta = rad2Deg(acos(((d3*d3) - (a*a) - (d2*d2)) / (-2 * a*d2)));
	double alpha1 = rad2Deg(asin(sin(deg2Rad(beta)) * (d2 / d3)));
	double alpha2 = rad2Deg(asin(py_dash / d3));


	

	double phi2_forward_downward, phi2_forward_upward;
	//if (d1Condition(_d1))
	//{
		phi2_forward_upward = -1.0 * (alpha2 + alpha1);
		phi2_forward_downward = -1.0 * (alpha2 - alpha1);
	/*}
	else
	{
		phi2_forward_upward = 180.0-1.0 * (alpha2 + alpha1);
		phi2_forward_downward = 180.0-1.0 * (alpha2 - alpha1);
	}*/

	double phi3_forward_upward = 360.0 - beta - rad2Deg(asin(b / d2)) - 90.0;
	double phi3_forward_downward = beta - rad2Deg(asin(b / d2)) - 90;

	static std::array<double, 4> ans = 
	{
		phi2_forward_upward,
		phi2_forward_downward,
		phi3_forward_upward,
		phi3_forward_downward
	};
	std::cout << "phi2_forward_upward " << phi2_forward_upward << endl;
	std::cout << "phi2_forward_downward " << phi2_forward_downward << endl;
	std::cout << "phi3_forward_upward " << phi3_forward_upward << endl;
	std::cout << "phi3_forward_downward " << phi3_forward_downward << endl;
	return &ans;
}

std::array<double, 4>* IVKinPos::backward_calc(double _d1, Position _wristPoint)
{
	std::cout << "backward_calc: " << endl;

	double px_dash = _d1 + m;
	double py_dash = _wristPoint.z - n;

	double d3 = sqrt(px_dash* px_dash + py_dash * py_dash);
	double d2 = sqrt(o*o + b * b);
	double beta = rad2Deg(acos(((d3*d3) - (a*a) - (d2*d2)) / (-2 * a*d2)));
	double alpha1 = rad2Deg(asin(sin(deg2Rad(beta)) * (d2 / d3)));
	double alpha2 = rad2Deg((asin(py_dash / d3)));
	
	double phi2_backward_upward, phi2_backward_downward;
	//if (d1Condition(_d1))
	//{
		phi2_backward_upward = (alpha2 + alpha1) - 180.0;
		phi2_backward_downward = (alpha2 - alpha1) - 180.0;
	//}
	/*else
	{
		phi2_backward_upward = -(alpha2 - alpha1);
		phi2_backward_downward = -(alpha2 - alpha1);
	}*/

	

	double phi3_backward_upward = -1.0 * (90.0 - 1.0 * (beta - rad2Deg(asin(b / d2))));
	double phi3_backward_downward = 360.0 - beta - rad2Deg(asin(b / d2)) - 90.0;
	double phi3_backward_downward_1 = 270.0 - beta - rad2Deg(asin(b / d2));
	static std::array<double, 4> ans =
	{
		phi2_backward_upward,
		phi2_backward_downward,
		phi3_backward_upward,
		phi3_backward_downward,
	};
	std::cout << "phi2_backward_upward " << phi2_backward_upward << endl;
	std::cout << "phi2_backward_downward " << phi2_backward_downward << endl;
	std::cout << "phi3_backward_upward " << phi3_backward_upward << endl;
	std::cout << "phi3_backward_downward " << phi3_backward_downward << endl;
	return &ans;
}


