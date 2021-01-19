#include "splines.h""

Splines::Splines()
{
	trajectory = new Trajectory();
	robot = &Robot::getInstance();
};
Trajectory * Splines::getSpline(vector<SixDPos*> &_points, double _velocity, double _acceleration)
{
	//normalsire Richtungvector und dann multipliziert mit velocitity
	//https://pomax.github.io/bezierinfo/#explanation
	//https://www.codeproject.com/Articles/31859/Draw-a-Smooth-Curve-through-a-Set-of-2D-Points-wit
	//http://blog.sklambert.com/finding-the-control-points-of-a-bezier-curve/
	//https://www.youtube.com/watch?v=P9XYzST6SZU
	//https://towardsdatascience.com/b%C3%A9zier-interpolation-8033e9a262c2
	double d;
	Position P_i, P_i1;
	vector<double>* b;
	vector<Position> points;

	std::vector<std::array<double, 2>*>* vel_acc;
	int n = _points.size();
	int numberOfSplines = n - 1;
	b->push_back(0.0); // StartPunkt
	std::array<double, 2> start_vel_acc = std::array<double, 2>{0.0, 0.0};
	vel_acc->push_back(&start_vel_acc);
	for (int i = 0; i < _points.size()-1; i++)
	{
		P_i = Position(_points.at(i)->get_X(), _points.at(i)->get_Y(), _points.at(i)->get_Z());
		P_i1 = Position(_points.at(i+1)->get_X(), _points.at(i+1)->get_Y(), _points.at(i+1)->get_Z());
		d = sqrt(pow(P_i.x - P_i1.x, 2) + pow(P_i.y - P_i1.y, 2) + pow(P_i.y - P_i1.y, 2));

		b->push_back(b->back() + d);


	}
	

	std::vector<Splines::Position> tangents = getTagents(points);
	std::vector<Splines::Position> accelerations = getAccelerations(points,tangents);

	vector<Configuration *> configs;
	double dt = robot->time_interval;
	double t = 0.5;
	int sample = 100;
	vector<double> distances;
	for (int i = 0; i < numberOfSplines; i++)
	{
		for (int j = 0; j < sample; j++)
		{
			//geeignete t finden
			Position p = getQuinticBezier(j / sample, i, points, tangents, accelerations);
			//inverse Kinematik
			//push
		}
	}
	
	trajectory->set_trajectory(configs);
}



std::vector<Splines::Position> Splines::getTagents(vector<Splines::Position>& _points)
{
	std::vector<Splines::Position> tagents;

	//TODO jetztige Konfig von Roboter
	
	double e = 0.5;
	int n = _points.size();
	for (int i = 1; i < n- 1; i++)
	{
		Position direction0 = _points.at(i) - _points.at(i - 1);
		Position direction1 = _points.at(i+1) - _points.at(i);

		tagents.push_back((direction0 / L2Norm(direction0) + direction1 / L2Norm(direction1)) * e * 0.5 * min(L2Norm(direction0), L2Norm(direction1)) * 0.5);
	}
	tagents.push_back(_points.at(n - 1) - _points.at(n - 2));
	if (tagents.size() != _points.size())
		std::cout << "falsch";
	return tagents;
}

std::vector<Splines::Position> Splines::getAccelerations(vector<Splines::Position>& _points, std::vector<Splines::Position> _tangents)
{
	//TODO letze und erste noch einfügen
	std::vector<Splines::Position> accelerations;
	return std::vector<Position>();
	int n = _points.size();
	double alpha, beta;
	Position A, B, C;
	Position Ta, Tb, Tc;

	for (int i = 1; i < n - 1; i++)
	{
		A = _points.at(i - 1);
		B = _points.at(i);
		C = _points.at(i + 1);

		Ta = _tangents.at(i - 1);
		Tb = _tangents.at(i);
		Tc = _tangents.at(i + 1);
		alpha = L2Norm(C - B) / (L2Norm(B - A) + L2Norm(C - B));
		beta = L2Norm(B - A) / (L2Norm(B - A) + L2Norm(C - B));

		if (i == 1)
		{
			accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 - B * 6));
		}
		accelerations.push_back((A * 6 + Ta * 2 + Tb * 4 - B * 6) * alpha + (B * (-6) - Tb * 4 - Tc * 2 + C * 6) * beta);

		if (i == (n - 2))
		{
			accelerations.push_back((B * (-6) - Tb * 4 - Tc * 2 + C * 6));
		}







	}
}



double Splines::L2Norm(Position _p)
{
	return sqrt(pow(_p.x,2) + pow(_p.y,2) + pow(_p.y, 2));
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


