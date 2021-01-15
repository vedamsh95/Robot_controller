#include "splines.h""

Trajectory * Splines::getQuinticBezier(vector<SixDPos*> &points, double _velocity, double _acceleration)
{
	//normalsire Richtungvector und dann multipliziert mit velocitity
	//https://pomax.github.io/bezierinfo/#explanation
	//https://www.codeproject.com/Articles/31859/Draw-a-Smooth-Curve-through-a-Set-of-2D-Points-wit
	//http://blog.sklambert.com/finding-the-control-points-of-a-bezier-curve/
	//https://www.youtube.com/watch?v=P9XYzST6SZU
	double d;
	Position P_i, P_i1;
	vector<double>* b;
	std::vector<std::array<double, 2>*>* vel_acc;

	b->push_back(0.0); // StartPunkt
	std::array<double, 2> start_vel_acc = std::array<double, 2>{0.0, 0.0};
	vel_acc->push_back(&start_vel_acc);
	for (int i = 0; i < points.size()-1; i++)
	{
		P_i = Position(points.at(i)->get_X(), points.at(i)->get_Y(), points.at(i)->get_Z());
		P_i1 = Position(points.at(i+1)->get_X(), points.at(i+1)->get_Y(), points.at(i+1)->get_Z());
		d = sqrt(pow(P_i.x - P_i1.x, 2) + pow(P_i.y - P_i1.y, 2) + pow(P_i.y - P_i1.y, 2));

		b->push_back(b->back() + d);


	}
	return nullptr;
}
