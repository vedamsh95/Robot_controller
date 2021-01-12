#include "TMatrix.h"
#include <math.h>

//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!

TMatrix::TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen) {
	
/**
 * Constructor of TMatrix with all given entries of the matrix
 * @param _one idx_0_0
 * @param _two idx_0_1
 * @param _three idx_0_2
 * @param _four idx_0_3
 * @param _five idx_1_0
 * @param _six idx_1_1
 * @param _seven idx_1_2
 * @param _eight idx_1_3
 * @param _nine idx_2_0
 * @param _ten idx_2_1
 * @param _eleven idx_2_2
 * @param _twelve idx_2_3
 * @param _thirteen idx_3_0
 * @param _fourteen idx_3_1
 * @param _fifteen idx_3_2
 * @param _sixteen idx_3_3
 */

	m_transformation[0][0] = _one;
	m_transformation[0][1] = _two;
	m_transformation[0][2] = _three;
	m_transformation[0][3] = _four;
	m_transformation[1][0] = _five;
	m_transformation[1][1] = _six;
	m_transformation[1][2] = _seven;
	m_transformation[1][3] = _eight;
	m_transformation[2][0] = _nine;
	m_transformation[2][1] = _ten;
	m_transformation[2][2] = _eleven;
	m_transformation[2][3] = _twelve;
	m_transformation[3][0] = _thirteen;
	m_transformation[3][1] = _fourteen;
	m_transformation[3][2] = _fifteen;
	m_transformation[3][3] = _sixteen;
}


TMatrix::TMatrix(double _trans[6]) {
	
/**
	 * Constructor for the the TMatrix set up with the passed array containing the rotation and translation of a position.
	 *  - _trans[0]: rotation around x-axis
	 *  - _trans[1]: rotation around y-axis
	 *  - _trans[2]: rotation around z-axis
	 *  - _trans[3]: translation in x direction
	 *  - _trans[4]: translation in y direction
	 *  - _trans[5]: translation in z direction
	 *
	 * @param _trans array containing rotation and position values
*/

	m_transformation[0][0] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[0][1] = sin(_trans[0]) * cos(_trans[1]);
	m_transformation[0][2] = -sin(_trans[1]);
	m_transformation[0][3] = 0; 
	m_transformation[1][0] = (cos(_trans[0]) * sin(_trans[1]) * sin(_trans[2])) - (sin(_trans[0]) * cos(_trans[2]));;
	m_transformation[1][1] = (sin(_trans[0]) * sin(_trans[1]) * sin(_trans[2])) + ((cos(_trans[0]) * cos(_trans[2])));
	m_transformation[1][2] = cos(_trans[1]) * sin(_trans[2]);
	m_transformation[1][3] = 0;
	m_transformation[2][0] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[2][1] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[2][2] = cos(_trans[0]) * cos(_trans[1]);
	m_transformation[2][3] = 0;
	m_transformation[3][0] = _trans[3];
	m_transformation[3][1] = _trans[4];
	m_transformation[3][2] = _trans[5];
	m_transformation[3][3] = 1;

}


TMatrix::TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z) {

/**
	 * Constructor for the the TMatrix set up with the rotation and translation of a position.
	 *
	 * @param _rot_x rotation around x-axis
	 * @param _rot_y rotation around y-axis
	 * @param _rot_z rotation around z-axis
	 * @param _trans_x translation in x direction
	 * @param _trans_y translation in y direction
	 * @param _trans_z translation in z direction
*/

	m_transformation[0][0] = cos(_rot_x) * cos(_rot_y);
	m_transformation[0][1] = sin(_rot_x) * cos(_rot_y);
	m_transformation[0][2] = -sin(_rot_y);
	m_transformation[0][3] = 0;
	m_transformation[1][0] = (cos(_rot_x) * sin(_rot_y) * sin(_rot_z)) - (sin(_rot_x) * cos(_rot_z));;
	m_transformation[1][1] = (sin(_rot_x) * sin(_rot_y) * sin(_rot_z)) + ((cos(_rot_x) * cos(_rot_z)));
	m_transformation[1][2] = cos(_rot_y) * sin(_rot_z);
	m_transformation[1][3] = 0;
	m_transformation[2][0] = cos(_rot_x) * cos(_rot_y);
	m_transformation[2][1] = cos(_rot_x) * cos(_rot_y);
	m_transformation[2][2] = cos(_rot_x) * cos(_rot_y);
	m_transformation[2][3] = 0;
	m_transformation[3][0] = _trans_x;
	m_transformation[3][1] = _trans_y;
	m_transformation[3][2] = _trans_z;
	m_transformation[3][3] = 1;
}

