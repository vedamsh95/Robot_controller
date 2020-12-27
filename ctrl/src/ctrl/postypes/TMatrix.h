#ifndef SDRI_CTRL2019_TMATRIX_H
#define SDRI_CTRL2019_TMATRIX_H

/** \brief a homogeneous transformation matrix
 *  This class represents a homogenous matrix generated by a position. This is only an example implementation using
 *  X-Y-Z rotation.
 *
 *  !!! For your own implementation you must reimplement this class.!!!
 */
class TMatrix {
private:
	double m_transformation[4][4];
	double tip_cam_trans[4][4];

public:

    TMatrix();
    TMatrix(double theta_n, double alpha_n, double r_n, double d_n);

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
	TMatrix(double _trans[6]);

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
	TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z);

	/**
	 * Constructor of TMatrix with all given entries of the matrix
	 * @param _one idx0_0_
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
    TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen);

	/**
	 * Getter for internal matrix representation
	 * @return reference of the internal matrix representation
	 */
	double* get_matrix() { return &m_transformation[0][0]; };

	double get_element(unsigned int row, unsigned int column);

	void output();

	TMatrix operator*(const TMatrix& mat1);





};


#endif //SDRI_CTRL2019_TMATRIX_H
