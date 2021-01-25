#ifndef SDRI_CTRL2019_TMATRIX_H
#define SDRI_CTRL2019_TMATRIX_H

class TMatrix {
private:
	double m_transformation[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double _trans[6] = {0,0,0,0,0,0};

public:

	TMatrix(double _trans[6]);
	TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z);
    TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen);
	void getTmatrix(double &(matrix));
};

#endif //SDRI_CTRL2019_TMATRIX_H
