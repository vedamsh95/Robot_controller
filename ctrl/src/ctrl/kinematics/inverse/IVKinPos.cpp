#include "IVKinPos.h"
#include<iostream>


IVKinPos::IVKinPos()
{
	robot = &Robot::getInstance();
	m = robot->m;
	n = robot->n;
	o = robot->o;
	a = robot->a;
	b = robot->b;

}
IVKinPos::~IVKinPos()
{
}


std::vector<std::array<double, 3>*>* IVKinPos::get_IVKinPos(SixDPos* _pos)
{
	//angle of first joint
	double phi1;
	std::array<double, 3> a_wristPoint = getWristCenterPoint(_pos);
	//store Wrist Coordinates in struct Position for better handling
	Position wristPoint = Position(a_wristPoint.at(0), a_wristPoint.at(1), a_wristPoint.at(2));
	
	//Handle overhead singularity if only one SixDPos is given (without interpolation) 
	if (wristPoint.x > -marginSingularity && wristPoint.x < marginSingularity &&
		wristPoint.y > -marginSingularity && wristPoint.y < marginSingularity)
	{
		//setting angle of first angle to default value
		phi1 = 90.0;
		std::cout << "Overhead Singularity" << endl;
	}
	else
		phi1 = -rad2Deg(atan2(wristPoint.y, wristPoint.x));

	return calc_configurations(phi1, wristPoint);
}

std::array<double, 3> IVKinPos::getWristCenterPoint(SixDPos * _pos)
{
	//Transformation matrix of tool center point 
	TMatrix m_transEndeffector = TMatrix(_pos->get_C(), _pos->get_B(), _pos->get_A(), _pos->get_X(), _pos->get_Y(), _pos->get_Z());
	std::array<double, 3> wristPoint = {0.0, 0.0, 0.0};
	
	wristPoint.at(0) = _pos->get_X() - d_6 * m_transEndeffector.get(0, 2);
	wristPoint.at(1) = _pos->get_Y() - d_6 * m_transEndeffector.get(1, 2);
	wristPoint.at(2) = _pos->get_Z() - d_6 * m_transEndeffector.get(2, 2);
	return wristPoint;
}


double IVKinPos::deg2Rad(double _deg)
{
	return _deg / (180.0 / M_PI);
}

double IVKinPos::rad2Deg(double _rad)
{
	return _rad * (180.0 / M_PI);
}

void IVKinPos::checkLimits(double _phi1, std::array<double, 4>* _solution, std::vector<std::array<double, 3>*>* _ans)
{
	std::array<double, 3>* config = new std::array<double, 3>();
	if (rad2Deg(robot->limits[0].min) < _phi1 && _phi1 < rad2Deg(robot->limits[0].max))
	{
		if (rad2Deg(robot->limits[1].min) < _solution->at(0) && _solution->at(0) < rad2Deg(robot->limits[1].max)) //
		{
			if (rad2Deg(robot->limits[2].min) < _solution->at(2) && _solution->at(2) < rad2Deg(robot->limits[2].max))
			{
				//Downward Configuration
				config = new std::array<double, 3>{ _phi1, _solution->at(0), _solution->at(2) };
				_ans->push_back(config);
			}
		}
		if (rad2Deg(robot->limits[1].min) < _solution->at(1) && _solution->at(1) < rad2Deg(robot->limits[1].max)) //
		{
			if (rad2Deg(robot->limits[2].min) < _solution->at(3) && _solution->at(3) < rad2Deg(robot->limits[2].max))
			{
				//Upward Configuration
				config = new std::array<double, 3>{ _phi1, _solution->at(1), _solution->at(3) };
				_ans->push_back(config);
			}
		}

	}
}




std::vector<std::array<double, 3>*>* IVKinPos::calc_configurations(double _phi1, Position _wristPoint)
{
	std::array<double, 4> forwardSolutions;
	std::array<double, 4> backwardSolutions;
	//
	std::vector<std::array<double, 3>*>* ans = new std::vector<std::array<double, 3>*>();

	double d1 = std::sqrt(_wristPoint.x * _wristPoint.x + _wristPoint.y * _wristPoint.y);

	forwardSolutions = forward_calc(d1, _wristPoint);

	checkLimits(_phi1, &forwardSolutions, ans);

	//second forward solutions
	if ((-185.0) <= _phi1 && _phi1 <= (-175.0))
	{
		checkLimits(360.0 + _phi1, &forwardSolutions, ans);
	}
	if ((185.0) >= _phi1 && _phi1 >= (175.0))
	{
		checkLimits(-(360.0 - _phi1), &forwardSolutions, ans);
	}

	backwardSolutions = backward_calc(d1, _wristPoint);

	
	if ((5) >= _phi1 && _phi1 >= (-5.0))
	{
		//two backwards solutions
		checkLimits(+(_phi1 + 180.0), &backwardSolutions, ans);
		checkLimits(-(180.0 - _phi1), &backwardSolutions, ans);

	}
	else
	{
		//point reflection at origin -> get backward angle of first joint
		//Alternative: check quadrant and + or - 180.0
		double phi1_backward = -rad2Deg(atan2(-_wristPoint.y, -_wristPoint.x));
		checkLimits(phi1_backward, &backwardSolutions, ans);
	}

	return ans;
}




bool IVKinPos::d1Condition(double _d1)
{
	return (_d1 >= m || _d1 == 0);
}

std::array<double, 4> IVKinPos::forward_calc(double _d1, Position _wristPoint)
{
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
	if (d1Condition(_d1) || py_dash < 0)
	{
		phi2_forward_upward = -1.0 * (alpha2 + alpha1);
		phi2_forward_downward = -1.0 * (alpha2 - alpha1);
	}
	else
	{
		phi2_forward_upward = -(180.0 - (alpha2 - alpha1));
		phi2_forward_downward = -(180.0 - (alpha2 + alpha1));
	}

	double phi3_forward_upward = 360 - beta - rad2Deg(asin(b / d2)) - 90.0;
	double phi3_forward_downward = beta - rad2Deg(asin(b / d2)) - 90;

	return std::array<double, 4> {
		phi2_forward_upward,
			phi2_forward_downward,
			phi3_forward_upward,
			phi3_forward_downward
	};
}

std::array<double, 4> IVKinPos::backward_calc(double _d1, Position _wristPoint)
{
	double px_dash = _d1 + m;
	double py_dash = _wristPoint.z - n;

	double d3 = sqrt(px_dash* px_dash + py_dash * py_dash);
	double d2 = sqrt(o*o + b * b);
	double beta = rad2Deg(acos(((d3*d3) - (a*a) - (d2*d2)) / (-2 * a*d2)));
	double alpha1 = rad2Deg(asin(sin(deg2Rad(beta)) * (d2 / d3)));
	double alpha2 = rad2Deg((asin(py_dash / d3)));

	double phi2_backward_upward, phi2_backward_downward;
	double phi3_backward_upward, phi3_backward_downward;
	if (d1Condition(_d1))
	{
		phi2_backward_upward = (alpha2 + alpha1) - 180.0;
		phi2_backward_downward = (alpha2 - alpha1) - 180.0;
	}
	else
	{
		//else and if same formulas
		phi2_backward_upward = -(180.0 - (alpha2 + alpha1));
		phi2_backward_downward = -(180 - (alpha2 - alpha1));

	}
	phi3_backward_upward = -(90.0 - (beta - rad2Deg(asin(b / d2))));
	phi3_backward_downward = 270.0 - beta - rad2Deg(asin(b / d2));

	std::array<double, 4> ans =
	{
			phi2_backward_upward,
			phi2_backward_downward,
			phi3_backward_upward,
			phi3_backward_downward,
	};
	return ans;
}


