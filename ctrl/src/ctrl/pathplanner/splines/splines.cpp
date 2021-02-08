#include "splines.h"

Splines::Splines() :plot(true)
{
	trajectory = new Trajectory();
	robot = &Robot::getInstance();
	ivMovement = new IVMovement();
	fwK = new FwKinematics();
	//Distance between points on straight line between two waypoints
	sampleDistance = 0.0005;
}
Splines::~Splines()
{
};
Trajectory * Splines::getSpline(vector<SixDPos*> &_points, Configuration * start_cfg,double _velocity, double _acceleration, std::vector<std::vector<SixDPos*>>* loopPoints, double _elong, int _spline_type)
{
	
	size_t n = _points.size();
	size_t numberOfSplines = n - 1;

	//only one point -> robot does not move
	if (numberOfSplines == 0) 
	{
		trajectory->add_configuration(start_cfg);
		return trajectory;
	}

	//vector with waypoint positions
	vector<Position> points;

	Position P_i, P_i1;
	//euclidean Distance between two waypoints
	double dBtwP;

	
	for (int i = 0; i < n; i++)
	{
		points.push_back(Position(_points.at(i)->get_X(), _points.at(i)->get_Y(), _points.at(i)->get_Z()));
		if (i > 0)
		{
			//Number of sample points dependent on distance between waypoints
			//get afterwards approximately equal distance between sample points
			P_i = Position(points.at(i).x, points.at(i).y, points.at(i).z);
			P_i1 = Position(points.at(i -1).x, points.at(i - 1).y, points.at(i - 1).z);
			dBtwP = L2Norm(P_i - P_i1);
			samples.push_back((dBtwP / sampleDistance));
		}
	}

	vector<Splines::Position>* secondControlPoints = new vector<Splines::Position>(n-1);
	vector<Splines::Position>* firstControlPoints = new vector<Splines::Position>(n-1);

	getCubicBezierControlPoints(points, firstControlPoints,secondControlPoints );

	
	vector<Splines::Position> cubicTagents(n);
	for(int i = 0; i< n-1; i++)
	{ 
		cubicTagents[i] = firstControlPoints->at(i) - points[i];
	}
	cubicTagents[n - 1] = points[n - 1] - secondControlPoints->at(n - 2);

	std::vector<Splines::Position> quniticTangents;
	std::vector<Splines::Position> accelerations = getAccelerationsQuintic(points,cubicTagents);
	double splineLength;
	vector<double> sampleDist;
	if (_spline_type == 0)
	{
		//Cubic samples
		sampleDistancesCubic(points, *firstControlPoints, *secondControlPoints, &sampleDist);
		splineLength = sampleDist.back();
	}
	else if(_spline_type == 1)
	{
		//Quintic samples
		quniticTangents = getTangentsQuintic(points, _elong);
		sampleDistancesQuintic(points, quniticTangents, accelerations,&sampleDist);
		splineLength = sampleDist.back();
	}
	 
	std::vector<double> tValues = getTValues(splineLength, _velocity, _acceleration);


	//filling vector to plot spline
	vector<Position> trajectPoints;
	//positions of trajectory (return values)
	vector<SixDPos*>* trajectSixDPos = new vector<SixDPos*>();

	//current point on spline
	Position tP;
	//index of sampled distances
	int index = 0;
	//number of spline seqment
	int nr = 0;
	//t-Value of seqment [0,1]
	double t;
	//current distance on spline
	double actDistance;
	//t-Values for plotting
	vector<double> t_vec;

	int sample = 0;
	for (int i = 0; i < tValues.size(); i++)
	{
		//find corresponding t value to requiered distance -> correct velocity at spline point
		actDistance = tValues[i] * splineLength;
		while (actDistance > sampleDist.at(index))
		{
			index++;
		}
		//transition to next seqment of spline
		if (index > (sample + samples[nr]))
		{
			
			sample = sample + samples[nr];
			nr++;
		}
		t = (double)(index - sample)/samples[nr];

		t_vec.push_back(t);

		if (nr == numberOfSplines)
		{
			break;	
		}
		else
		{
			if (_spline_type == 0)
			{
				tP = getCubicBezier(t, nr, points, firstControlPoints, secondControlPoints);
			}
			else if (_spline_type == 1)
			{
				tP = getQuinticBezier(t, nr, points, quniticTangents, accelerations);
			}
		}

			
		trajectPoints.push_back(tP);
		
		trajectSixDPos->push_back(new SixDPos(tP.x, tP.y, tP.z, _points.at(0)->get_A(), _points.at(0)->get_B(), _points.at(0)->get_C()));
	}
	//last point
	trajectPoints.push_back(points[n - 1]);
	trajectSixDPos->push_back(new SixDPos(points[n - 1].x, points[n - 1].y, points[n - 1].z, _points.at(0)->get_A(), _points.at(0)->get_B(), _points.at(0)->get_C()));


	if (plot)
		plotSpline(trajectPoints, t_vec);
	
	//find corresponding joint configurations to trajectory
	trajectory = ivMovement->getMovement(trajectSixDPos, start_cfg, loopPoints);


	return 	trajectory;
}



void Splines::plotSpline(vector<Splines::Position>& _points, vector<double> _tValues)
{
#ifdef PLOT
	std::cout << "plotSpline" << endl;

	std::vector<double> x, y, z;

	for (int i = 0; i < _points.size(); i++) {
		x.push_back(_points[i].x);
		y.push_back(_points[i].y);
		z.push_back(_points[i].z);
	}

	std::map<std::string, std::string> keywords;
	keywords.insert(std::pair<std::string, std::string>("label", "spline curve"));
	plt::plot3(x, y, z, keywords);


	plt::xlabel("x label");
	plt::ylabel("y label");
	plt::set_zlabel("z label");
	plt::legend();
	plt::show();

	std::vector<double> t(_points.size());
	for (int i = 0; i < t.size(); i++) {
		t.at(i) = i * Robot::getInstance().time_interval;
	}

	std::vector<double> velocities(_points.size());
	for (int i = 0; i < velocities.size(); i++)
	{
		if (i == 0) {
			velocities.at(i) =0 ;
		}
		else {
			velocities.at(i) = L2Norm(_points.at(i) - _points.at(i-1)) / Robot::getInstance().time_interval;
		}
	}
	keywords.clear();
	keywords.insert(std::pair<std::string, std::string>("label", "velocities"));
	plt::plot(t,velocities, keywords);


	plt::xlabel("Time");
	plt::ylabel("velocity");
	plt::legend();
	plt::show();

	std::vector<double> index(_tValues.size());
	for (int i = 0; i < _tValues.size(); i++)
	{
		index.at(i) = i;
	}
	keywords.clear();
	keywords.insert(std::pair<std::string, std::string>("label", "tValues"));
	plt::plot(index, _tValues, keywords);


	plt::xlabel("index");
	plt::ylabel("tValues");
	plt::legend();
	//plt::show();

	std::cout << "plotSpline - Done" << endl;
#else
	std::cout << "The plotting functionality of configurations is disabled in the cmake script!" << std::endl;
#endif
}

std::vector<Splines::Position> Splines::getTangentsQuintic(vector<Splines::Position>& _points, double _elong)
{
	std::vector<Splines::Position> tangents;
	double length;
	Position direction0, direction1, tangent;
	size_t n = _points.size();
	if(n == 2)
	{
		//straight line
		direction0 = _points.at(1) - _points.at(0);
		length = L2Norm(direction0) * _elong;
		direction0 = direction0 / L2Norm(direction0);
		tangents.push_back(direction0 * length);
		tangents.push_back(direction0 * -length);
		return tangents;
		
	}

	for (int i = 1; i < n - 1; i++)
	{
		direction0 = (_points.at(i) - _points.at(i - 1));
		direction1 = (_points.at(i + 1) - _points.at(i));
		length = min(L2Norm(direction0), L2Norm(direction1)) * _elong;


		direction0 = direction0 / L2Norm(direction0);
		direction1 = direction1 / L2Norm(direction1);

		if (i == 1)
		{
			tangent = direction0 * (-1) + direction1;
			tangent = (tangent *(-1)) / L2Norm(tangent);
			tangents.push_back(tangent * length);
		}
		tangent = (direction0 + direction1);
		tangent = tangent / L2Norm(tangent);
		tangents.push_back(tangent * length);
	}
	tangent = direction0 * (-1) + direction1;
	tangent = (tangent) / L2Norm(tangent);
	tangents.push_back(tangent * length);
	
	return tangents;

}

void Splines::getCubicBezierControlPoints(vector<Splines::Position>& _points, vector<Splines::Position>* _firstControlPoints, vector<Splines::Position>* _secondControlPoints)
{
	size_t n = _points.size()-1;
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

std::vector<Splines::Position> Splines::getAccelerationsQuintic(vector<Splines::Position>& _points, std::vector<Splines::Position> _tangents)
{

	std::vector<Splines::Position> accelerations;

	size_t n = _points.size();
	double alpha, beta;
	Position A, B, C;
	Position Ta, Tb, Tc;
	if (n == 2)
	{
		accelerations.push_back(_tangents.at(0) / 3);
		accelerations.push_back(_tangents.at(1) / 3);
	}
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

		if (i == 1)//first
		{
			accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 + B * (-6)));
		}
		accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 + B * (-6)) * alpha + (B * (-6) + Tb * (-4) + Tc * (-2) + C * 6) * beta);

		if (i == (n - 2))//last
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
	return 
		_points.at(_i) * pow(1 - _t, 3) +
		_firstControlPoints->at(_i) * 3* pow(1 - _t, 2) * _t+
		_secondControlPoints->at(_i) * 3* (1- _t) * pow(_t,2)+ 
		_points.at(_i +1) * pow(_t, 3);
}

void Splines::sampleDistancesCubic(std::vector<Position> _points, std::vector<Position> _firstControlPoints, std::vector<Position> _secondControlPoints, vector<double>* _sampleDist)
{
	size_t numberOfSplines = _points.size() - 1;
	Position P_0, P_1;

	_sampleDist->push_back(0);
	P_0 = _points.at(0);
	for (int i = 0; i < numberOfSplines; i++)
	{
		for (int j = 0; j < samples[i]; j++)
		{
			double t = (double)j / samples[i];
			P_1 = getCubicBezier(t, i, _points, &_firstControlPoints, &_secondControlPoints);

			_sampleDist->push_back(_sampleDist->back() + L2Norm(P_1 - P_0));
			P_0 = P_1;
		}
	}
	
	
}

void Splines::sampleDistancesQuintic(std::vector<Position> _points, std::vector<Position> _tangents, std::vector<Position> _accelerations, vector<double>* _sampleDist)
{
	size_t numberOfSplines = _points.size() - 1;
	Position P_0, P_1;

	_sampleDist->push_back(0);
	P_0 = _points.at(0);
	for (int i = 0; i < numberOfSplines; i++)
	{
		for (int j = 0; j < samples[i]; j++)
		{
			double t = (double)j / samples[i];
			P_1 = getQuinticBezier(t, i, _points, _tangents, _accelerations);

			_sampleDist->push_back(_sampleDist->back() + L2Norm(P_1 - P_0));
			P_0 = P_1;
		}
	}
}

vector<double> Splines::getTValues(double _distance, double _velocity, double _acceleration)
{
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
	size_t cycles = (size_t)roundf(tmp)+3;


	//calculate SixDPoses for the Trajectory
	vector<double> tValues;
	double value, factor;
	double t = 0;
	vector<SixDPos*> points;
	for (size_t c = 0; c < cycles; c++) {
		value = StepTrajectory->eval(t);
		factor = value / _distance;
		tValues.push_back(factor);
		t += Robot::getInstance().time_interval;
	}
	return tValues;
}



