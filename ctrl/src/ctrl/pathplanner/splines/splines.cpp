#include "splines.h""

Splines::Splines() :plot(false)
{
	trajectory = new Trajectory();
	robot = &Robot::getInstance();
	ivMovement = new IVMovement();
	fwK = new FwKinematics();
}
Splines::~Splines()
{
}
;
Trajectory * Splines::getSpline(vector<SixDPos*> &_points, Configuration * start_cfg,double _velocity, double _acceleration, int _spline_type)
{
	std::cout << "getSpline" << endl;
	double d;
	Position P_i, P_i1;
	vector<double>* b;
	vector<Position> points;

	
	for (int i = 0; i < _points.size(); i++)
	{
		points.push_back(Position(_points.at(i)->get_X(), _points.at(i)->get_Y(), _points.at(i)->get_Z()));
	}

	int n = points.size();
	int numberOfSplines = n - 1;


	vector<Splines::Position>* secondControlPoints = new vector<Splines::Position>(n-1);
	vector<Splines::Position>* firstControlPoints = new vector<Splines::Position>(n-1);;
	getCubicBezierControlPoints(points, firstControlPoints,secondControlPoints );
	Position p = getCubicBezier(0.5, 0, points, firstControlPoints, secondControlPoints);
	vector<Splines::Position> cubicTagents(n);
	for(int i = 0; i< n-1; i++)
	{ 
		cubicTagents[i] = firstControlPoints->at(i) - points[i];
	}
	cubicTagents[n - 1] = points[n - 1] - secondControlPoints->at(n - 2);

	std::vector<Splines::Position> tangents;
	std::vector<Splines::Position> accelerations = getAccelerations(points,cubicTagents);
	double distance;
	if (_spline_type == 0)
	{
		
		distance = getSplineLengthCubic(points, *firstControlPoints, *secondControlPoints);
	}
	else if(_spline_type == 1)
	{
		tangents = getTagents(points);
		distance = getQuniticSplineLength(points, tangents, accelerations);

	}
	 
	//
	std::vector<double> tValues = getTValues(distance, _velocity, _acceleration);

	int nr;
	double t,v;
	vector<Position> trajectPoints;
	vector<SixDPos*>* trajectSixDPos = new vector<SixDPos*>();
	vector<double >x;
	vector<double >y;
	vector<double >z;
	SixDPos* pos;
	vector<Configuration *> configs;
	Position tP;
	for (int i = 0; i < tValues.size(); i++)
	{
		v = tValues[i] * numberOfSplines;
		nr = (int) v;
		t = v - nr;
		if (nr < numberOfSplines)
		{
			if (_spline_type == 0)
			{
				tP = getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints);
			}
			else if (_spline_type == 1)
			{
				tP = getQuinticBezier(t, nr, points, tangents, accelerations);
			}
			//trajectPoints.push_back(getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints));
			//x.push_back(getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints).x);
			//y.push_back(getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints).y);
			//z.push_back( getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints).z);
			
			trajectPoints.push_back(tP);
			//configs.push_back(invK->get_inv_kinematics(new SixDPos(tP.x, tP.y, tP.z, _points.at(0)->get_A(), _points.at(0)->get_B(), _points.at(0)->get_C())));
			trajectSixDPos->push_back(new SixDPos(tP.x, tP.y, tP.z, _points.at(0)->get_A(), _points.at(0)->get_B(), _points.at(0)->get_C()));
			//x.push_back(getQuinticBezier(t, nr, points, tangents, accelerations).x);
			//y.push_back(getQuinticBezier(t, nr, points, tangents, accelerations).y);
			//z.push_back(getQuinticBezier(t, nr, points, tangents, accelerations).z);
		}
			

	}
	
	trajectPoints.push_back(points[n - 1]);
	trajectSixDPos->push_back(new SixDPos(points[n - 1].x, points[n - 1].y, points[n - 1].z, _points.at(0)->get_A(), _points.at(0)->get_B(), _points.at(0)->get_C()));


	if (plot)
		plotSpline(trajectPoints);
	
	 
	double dt = robot->time_interval;
	trajectPoints.clear();
	Configuration * cfg;
	trajectory = ivMovement->getMovement(trajectSixDPos, start_cfg);
	for (int i = 0; i < trajectory->get_all_configuration()->size(); i++)
	{
		cfg = trajectory->get_all_configuration()->at(i);
		pos = fwK->get_fw_kinematics(cfg);
		trajectPoints.push_back(Position(pos->get_X(), pos->get_Y(), pos->get_Z()));
	}


	if (plot)
		plotSpline(trajectPoints);

	return 	trajectory;
}



void Splines::plotSpline(vector<Splines::Position>& _points)
{
#ifdef PLOT
	std::cout << "plotSpline" << endl;

	std::vector<double> x, y, z;

	for (double i = 0; i < _points.size(); i++) {
		x.push_back(_points[i].x);
		y.push_back(_points[i].y);
		z.push_back(_points[i].z);
	}

	std::map<std::string, std::string> keywords;
	keywords.insert(std::pair<std::string, std::string>("label", "parametric curve"));
	plt::plot3(x, y, z, keywords);


	plt::xlabel("x label");
	plt::ylabel("y label");
	plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
	plt::legend();
	plt::show();
	std::cout << "plotSpline - Done" << endl;
#else
	std::cout << "The plotting functionality of configurations is disabled in the cmake script!" << std::endl;
#endif
}

std::vector<Splines::Position> Splines::getTagents(vector<Splines::Position>& _points)
{
	std::cout << "getTagents" << endl;
	std::vector<Splines::Position> tagents;

	//TODO jetztige Konfig von Roboter
	
	double e = 0.5;
	double length;
	Position direction0, direction1, tangent;
	int n = _points.size();
	//tagents.push_back(_startTagent);
	if (n > 2)
	{
		for (int i = 1; i < n - 1; i++)
		{
			direction0 = (_points.at(i) - _points.at(i - 1));
			direction1 = (_points.at(i + 1) - _points.at(i));
			length = min(L2Norm(direction0), L2Norm(direction1)) * e;


			direction0 = direction0 / L2Norm(direction0);
			direction1 = direction1 / L2Norm(direction1);

			//double a = L2Norm(direction0);
			//double b = L2Norm(direction1);


			if (i == 1)
			{
				tangent = direction0 * (-1) + direction1;
				tangent = (tangent *(-1)) / L2Norm(tangent);//*-1
				tagents.push_back(tangent * length);
			}
			tangent = (direction0 + direction1);
			tangent = tangent / L2Norm(tangent);
			tagents.push_back(tangent * length);
			//tagents.push_back((direction0 / L2Norm(direction0) + direction1 / L2Norm(direction1))/pNorm * e * min(L2Norm(direction0), L2Norm(direction1)));//*0.5*0.5
		}
		tangent = direction0 * (-1) + direction1;
		tangent = (tangent) / L2Norm(tangent);
		tagents.push_back(tangent * length);
	}

	//Position p_1 = _points.at(n - 1);
	//Position p_2 = _points.at(n - 2);
	//Position test = p_1 - p_2;
	//
	//Position endTagent = (_points.at(n - 1) - _points.at(n - 2)) * 0.5;
	//tagents.push_back(endTagent);
	if (tagents.size() != _points.size())//?
		std::cout << "falsch";
	return tagents;

}

void Splines::getCubicBezierControlPoints(vector<Splines::Position>& _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints)
{
	//std::cout << "getCubicBezierControlPoints" << endl;
	int n = _points.size()-1;
	if (n < 1)
		return;
	if (n == 1)
	{ 
		_firstControlPoints->at(0) = (_points[0] *2 + _points[1]) / 3;
		_secondControlPoints->at(0) = _firstControlPoints->at(0) * 2 - _points[0];
		return;
	}
	vector<Position> rhs = vector<Position>(n);

	for (int i = 1; i < n - 1; ++i)
	{
		rhs[i] = _points[i] * 4 + _points[i + 1] * 2;
	}
	rhs[0] = _points[0] + _points[1] * 2;
	rhs[n - 1] = (_points[n - 1] * 8 + _points[n]) / 2.0;//
	//GetFirstControlPoints
	vector<Position> tmp = vector<Position>(n);

	Position b = Position(2.0,2.0,2.0);
	_firstControlPoints->at(0) = rhs[0] / b;
	for (int i = 1; i < n; i++) // Decomposition and forward substitution.
	{
		tmp[i] = Position(1)/b;
		b = Position(i < n - 1 ? 4.0 : 3.5) - tmp[i];
		_firstControlPoints->at(i) = (rhs[i] - _firstControlPoints->at(i - 1)) / b;
	}
	for (int i = 1; i < n; i++)
		_firstControlPoints->at(n - i - 1) = _firstControlPoints->at(n - i - 1) - (tmp[n - i] * _firstControlPoints->at(n - i)); // Backsubstitution.
	
	for (int i = 0; i < n; ++i)
	{

	// Second control point
	if (i < n - 1)
		_secondControlPoints->at(i) =  _points[i + 1] * 2 -_firstControlPoints->at(i + 1);
	else
		_secondControlPoints->at(i) = (_points[n] +  _firstControlPoints->at(n-1))/2;
	}


}

std::vector<Splines::Position> Splines::getAccelerations(vector<Splines::Position>& _points, std::vector<Splines::Position> _tangents)
{
	//std::cout << "getAccelerations" << endl;

	std::vector<Splines::Position> accelerations;

	int n = _points.size();
	double alpha, beta;
	Position A, B, C;
	Position Ta, Tb, Tc;

	for (int i = 1; i < n - 1; i++)
	{
		A = _points.at(i - 1);
		B = _points.at(i);
		C = _points.at(i + 1);

		Ta = _tangents.at(i - 1) ;
		Tb = _tangents.at(i);
		Tc = _tangents.at(i + 1);
		alpha = L2Norm(C - B) / (L2Norm(B - A) + L2Norm(C - B));
		beta = L2Norm(B - A) / (L2Norm(B - A) + L2Norm(C - B));

		if (i == 1)//erste
		{
			accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 + B * (-6)));
		}
		accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 + B * (-6)) * alpha + (B * (-6) + Tb * (-4) + Tc * (-2) + C * 6) * beta);

		if (i == (n - 2))//letzte 
		{
			accelerations.push_back((B * (-6) + Tb * (-4) + Tc * (-2) + C * 6));
		}
	}
	return accelerations;
}



double Splines::L2Norm(Position _p)
{
	return sqrt(pow(_p.x,2) + pow(_p.y,2) + pow(_p.z, 2));
}

Splines::Position Splines::getQuinticBezier(double _t, int _i, std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations)
{
	//std::cout << "getQuinticBezier" << endl;
	Position P1 = _tangents.at(_i) * 1 / 5 + _points.at(_i);
	Position P2 = _accelerations.at(_i) * 1 / 20 + P1 * 2 - _points.at(_i);
	Position P4 = _points.at(_i + 1) - _tangents.at(_i + 1) * 1 / 5;
	Position P3 = _accelerations.at(_i+1) * 1 / 20 + P4 * 2 - _points.at(_i+1);
	return 
		_points.at(_i) * pow(1 - _t, 5) +
		P1 * 5 * pow(1 - _t, 4) * _t +
		P2 * 10 * pow(1 - _t, 3)* pow(_t, 2) +
		P3 * 10 * pow(1 - _t, 2) * pow(_t, 3) +
		P4 * 5 * (1 - _t) * pow(_t, 4) +
		_points.at(_i + 1) * pow(_t, 5) ;
}

Splines::Position Splines::getCubicBezier(double _t, int _i, std::vector<Position> _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints)
{
	//std::cout << "getCubicBezier"<<endl;
	return 
		_points.at(_i) * pow(1 - _t, 3) +
		_firstControlPoints->at(_i) * 3* pow(1 - _t, 2) * _t+
		_secondControlPoints->at(_i) * 3* (1- _t) * pow(_t,2)+ 
		_points.at(_i +1) * pow(_t, 3);
}

double Splines::getQuniticSplineLength(std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations)
{
	//std::cout << "getSplineLength" << endl;
	int sample = 100;
	double distance = 0;
	int numberOfSplines = _points.size() - 1;
	Position P_0 = _points.at(0);
	for (int i = 0; i < numberOfSplines; i++)
	{
		for (int j = 0; j < sample; j++)
		{
			double t = (double)j / sample;
			Position P_1= getQuinticBezier(t, i, _points, _tangents, _accelerations);

			distance += L2Norm(P_1 - P_0);
			P_0 = P_1;
		}
	}
	return distance;
}

double Splines::getSplineLengthCubic(std::vector<Position> _points, std::vector<Position> _firstControlPoints, std::vector<Position> _secondControlPoints)
{
	//std::cout << "getSplineLengthCubic" << endl;
	double sample = 100;
	double distance = 0;
	int numberOfSplines = _points.size() - 1;
	Position P_0, P_1;
	P_0 = _points.at(0);
	for (int i = 0; i < numberOfSplines; i++)
	{
		for (int j = 0; j < sample; j++)
		{
			double t = (double) j / sample;
			 P_1= getCubicBezier(t, i, _points, &_firstControlPoints, &_secondControlPoints);

			distance += L2Norm(P_1 - P_0);
			P_0 = P_1;
		}
	}
	return distance;
}

vector<double> Splines::getTValues(double _distance, double _velocity, double _acceleration)
{
	//std::cout << "getTValues" << endl;
	//Determine whether a max. velocity profile
//is sufficient or a trapezoidal trajectory is needed.
	Single_trajectory::Type type = Single_trajectory::select_type(_distance,
		_velocity,
		_acceleration);

	//Get Trajectory of distances from the start pos.
	Single_trajectory* StepTrajectory;
	if (type == Single_trajectory::Type::MAX_VEL) {
		StepTrajectory = new Max_vel_trajectory(_velocity,
			_acceleration,
			0,
			_distance);
	}
	else if (type == Single_trajectory::Type::TRAPEZOIDAL) {
		StepTrajectory = new Trapezoidal_trajectory(_velocity,
			_acceleration,
			0,
			_distance);
	}


	auto tmp = static_cast<float> (StepTrajectory->get_duration() / Robot::getInstance().time_interval);
	size_t cycles = roundf(tmp) + 3;


	//calculate SixDPoses for the Trajectory
	vector<double> tValues;
	double value, factor;
	double t = 0;
	vector<SixDPos*> points;
	SixDPos* point;
	for (size_t c = 0; c < cycles; c++) {
		value = StepTrajectory->eval(t);
		factor = value / _distance;
		tValues.push_back(factor);
		t += Robot::getInstance().time_interval;
	}
	return tValues;
}



