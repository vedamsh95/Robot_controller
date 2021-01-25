#include "TMatrix.h"
#include <math.h>
#include <iostream>
using namespace std;

//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!

TMatrix::TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen) {
	m_transformation[0] = _one;
	m_transformation[1] = _two;
	m_transformation[2] = _three;
	m_transformation[3] = _four;
	m_transformation[4] = _five;
	m_transformation[5] = _six;
	m_transformation[6] = _seven;
	m_transformation[7] = _eight;
	m_transformation[8] = _nine;
	m_transformation[9] = _ten;
	m_transformation[10] = _eleven;
	m_transformation[11] = _twelve;
	m_transformation[12] = _thirteen;
	m_transformation[13] = _fourteen;
	m_transformation[14] = _fifteen;
	m_transformation[15] = _sixteen;
}


TMatrix::TMatrix(double _trans[6]) 
{	
	m_transformation[0] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[1] = sin(_trans[0]) * cos(_trans[1]);
	m_transformation[2] = -sin(_trans[1]);
	m_transformation[3] = 0; 
	m_transformation[4] = (cos(_trans[0]) * sin(_trans[1]) * sin(_trans[2])) - (sin(_trans[0]) * cos(_trans[2]));;
	m_transformation[5] = (sin(_trans[0]) * sin(_trans[1]) * sin(_trans[2])) + ((cos(_trans[0]) * cos(_trans[2])));
	m_transformation[6] = cos(_trans[1]) * sin(_trans[2]);
	m_transformation[7] = 0;
	m_transformation[8] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[9] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[10] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[11] = 0;
	m_transformation[12] = _trans[3];
	m_transformation[13] = _trans[4];
	m_transformation[14] = _trans[5];
	m_transformation[15] = 1;

}


TMatrix::TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z)
{


	//sending e1, e2, e3 (first 3 euler angles) Xp, Yp, Zp (final position) in order
	m_transformation[0] = cos(_rot_x) * cos(_rot_y);
	m_transformation[1] = (cos(_rot_x) * sin(_rot_y) * sin(_rot_z)) - (sin(_rot_x) * cos(_rot_z));
	m_transformation[2] = (sin(_rot_x)*sin(_rot_z))+(cos(_rot_x)* sin(_rot_y)* cos(_rot_z));
	m_transformation[3] = _trans_x;
	m_transformation[4] = sin(_rot_x) * cos(_rot_y);
	m_transformation[5] = (sin(_rot_x) * sin(_rot_y) * sin(_rot_z)) + ((cos(_rot_x) * cos(_rot_z)));
	m_transformation[6] = -(cos(_rot_x) * sin(_rot_z)) + (sin(_rot_x) * sin(_rot_y) * cos(_rot_z));
	m_transformation[7] = _trans_y;
	m_transformation[8] = -sin(_rot_y);
	m_transformation[9] = cos(_rot_y) * sin(_rot_z);
	m_transformation[10] = cos(_rot_y) * cos(_rot_z);
	m_transformation[11] = _trans_z;
	m_transformation[12] = 0;
	m_transformation[13] = 0;
	m_transformation[14] = 0;
	m_transformation[15] = 1;
}

void TMatrix::getTmatrix(double &(matrix))
{
	double* temp = &matrix;
	int i = 0;
	for (i = 0; i < 16; i++) 
	{
		*temp = m_transformation[i];
		temp++;
	}

}