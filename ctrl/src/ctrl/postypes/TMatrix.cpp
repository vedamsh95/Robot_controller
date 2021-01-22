#include "TMatrix.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include<iostream>
#include <array>
#include <ConfigProvider.h>
//#undef __STRICT_ANSI__


//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!
TMatrix::TMatrix() {}

//double M_PI = 3.1415926535897932384626433832795028841971; // just for testing

/*constructor homogeneous TranslationMatrix DenavitHartenberg TMatrix
 *@ requires:   theta   : rotation around the z-axis
 *              alpha_n : rotation around the x-axis
 *              r_n     : Distance of the origin n-1 to origin n along the x-axis of CSn (CS1) in millimeters
 *              d_n     : Distance of the origins n-1 to origin n along the z-axis of CSn-1 in millimeters
 *
 *@ ensures:    Object TMatrix 4x4 TranslationMatrix
 * */
TMatrix::TMatrix(double theta_n, double alpha_n, double r_n, double d_n) {              // new constructor Translationmatrix Denavit Hartenberg
    double const fa = M_PI/180;

    for (int h = 0; h < 4; h++)
    {
        for (int w = 0; w < 4; w++)
        {
            if(h == 0 && w == 0){
               m_transformation[h][w] = cos(theta_n*fa);                           //element one of hartenberg transformation matrix
            }
            else if (h==0 && w == 1){
                m_transformation[h][w] = -sin(theta_n*fa)*cos(alpha_n*fa);          //element two of hartenberg transformation matrix
            }
            else if (h==0 && w == 2){
                m_transformation[h][w] = sin(theta_n*fa)*sin(alpha_n*fa);           //element three of hartenberg transformation matrix
            }
            else if (h==0 && w == 3){
                m_transformation[h][w] = r_n*cos(theta_n*fa);                       //element four of hartenberg transformation matrix
            }
            else if(h == 1 && w == 0){
                m_transformation[h][w] = sin(theta_n*fa);                           //element five of hartenberg transformation matrix
            }
            else if (h==1 && w == 1){
                m_transformation[h][w] = cos(theta_n*fa)*cos(alpha_n*fa);           //element six of hartenberg transformation matrix
            }
            else if (h==1 && w == 2){
                m_transformation[h][w] = -cos(theta_n*fa)*sin(alpha_n*fa);          //element seven of hartenberg transformation matrix
            }
            else if (h==1 && w == 3){
                m_transformation[h][w] = r_n*sin(theta_n*fa);                       //element eight of hartenberg transformation matrix
            }
            else if(h == 2 && w == 0){
                m_transformation[h][w] = 0;                                         //element nine of hartenberg transformation matrix
            }
            else if (h==2 && w == 1){
                m_transformation[h][w] = sin(alpha_n*fa);                           //element ten of hartenberg transformation matrix
            }
            else if (h==2 && w == 2){
                m_transformation[h][w] = cos(alpha_n*fa);                           //element eleven of hartenberg transformation matrix
            }
            else if (h==2 && w == 3){
                m_transformation[h][w] = d_n;                                       //element twelve of hartenberg transformation matrix
            }
            else if(h == 3 && w == 0){
                m_transformation[h][w] = 0;                                         //element thirteen of hartenberg transformation matrix
            }
            else if (h==3 && w == 1){
                m_transformation[h][w] = 0;                                         //element fourteen of hartenberg transformation matrix
            }
            else if (h==3 && w == 2){
                m_transformation[h][w] = 0;                                         //element fifeteen of hartenberg transformation matrix
            }
            else if (h==3 && w == 3){
                m_transformation[h][w] = 1;                                         //element sixteen of hartenberg transformation matrix
            }
        }
    }
}



/*constructor  TMatrix
 *@ requires:   one     : double number
 *              two     : double number
 *              [...]   : double number
 *              sixteen : double number
 *
 *@ ensures:    Object general TMatrix 4x4
 * */
TMatrix::TMatrix(double _one, double _two, double _three, double _four, double _five, double _six, double _seven, double _eight, double _nine, double _ten, double _eleven, double _twelve, double _thirteen, double _fourteen, double _fifteen, double _sixteen) {
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

TMatrix::TMatrix(std::array<double,16> arr) {
    m_transformation[0][0] = arr[0];
    m_transformation[0][1] = arr[1];
    m_transformation[0][2] = arr[2];
    m_transformation[0][3] = arr[3];
    m_transformation[1][0] = arr[4];
    m_transformation[1][1] = arr[5];
    m_transformation[1][2] = arr[6];
    m_transformation[1][3] = arr[7];
    m_transformation[2][0] = arr[8];
    m_transformation[2][1] = arr[9];
    m_transformation[2][2] = arr[10];
    m_transformation[2][3] = arr[11];
    m_transformation[3][0] = arr[12];
    m_transformation[3][1] = arr[13];
    m_transformation[3][2] = arr[14];
    m_transformation[3][3] = arr[15];
}

/*constructor transformation matrix of denavit hartenberg method
 *@ requires:   double array[6] -->
 *
 *
 *@ ensures:    Object general TMatrix 4x4
 * */


//Matrix is transposed!!
TMatrix::TMatrix(double _trans[6]) {
    m_transformation[0][0] = cos(_trans[3])*cos(_trans[4]);
    m_transformation[1][0] = (-1)*sin(_trans[3])*cos(_trans[5]) + cos(_trans[3])*sin(_trans[4])*sin(_trans[5]);
    m_transformation[2][0] = sin(_trans[3])*sin(_trans[5])+cos(_trans[3])*sin(_trans[4])*cos(_trans[5]);
    m_transformation[3][0] = _trans[0];

    m_transformation[0][1] = sin(_trans[3])*cos(_trans[4]);
    m_transformation[1][1] = cos(_trans[3]) * cos(_trans[5])+sin(_trans[3])*sin(_trans[4])*sin(_trans[5]);
    m_transformation[2][1] = (-1)*cos(_trans[3])*sin(_trans[5]) + sin(_trans[3])*sin(_trans[4])*cos(_trans[5]);
    m_transformation[3][1] = _trans[1];

    m_transformation[0][2] = (-1)*sin(_trans[4]);
    m_transformation[1][2] = cos(_trans[4])*sin(_trans[5]);
    m_transformation[2][2] = cos(_trans[4]) * cos(_trans[5]);
    m_transformation[3][2] = _trans[2];

    m_transformation[0][3] = 0;
    m_transformation[1][3] = 0;
    m_transformation[2][3] = 0;
    m_transformation[3][3] = 1;

}




/*constructor  Rotational TMatrix
 *@ requires:   _rot_z     : rotation around the z-axis ---> phi in radian
 *              _rot_y     : rotation around the y-axis ---> theta in radian
 *              _rot_x     : rotation around the x-axis ---> psi in radian
 *              _trans_x   : double length translation_x in mm
 *              _trans_y   : double length translation_y in mm
 *              _trans_z   : double length translation_z in mm
 *@ ensures:    Object Rotational TMatrix 4x4
 * */
TMatrix::TMatrix(double _rot_z, double _rot_y, double _rot_x, double _trans_x, double _trans_y, double _trans_z) {
m_transformation[0][0] = cos(_rot_z)*cos(_rot_y);
m_transformation[0][1] = (-1)*sin(_rot_z)*cos(_rot_x) + cos(_rot_z)*sin(_rot_y)*sin(_rot_x);
m_transformation[0][2] = sin(_rot_z)*sin(_rot_x)+cos(_rot_z)*sin(_rot_y)*cos(_rot_x);
m_transformation[0][3] = _trans_x;

m_transformation[1][0] = sin(_rot_z)*cos(_rot_y);
m_transformation[1][1] = cos(_rot_z) * cos(_rot_x)+sin(_rot_z)*sin(_rot_y)*sin(_rot_x);
m_transformation[1][2] = (-1)*cos(_rot_z)*sin(_rot_x) + sin(_rot_z)*sin(_rot_y)*cos(_rot_x);
m_transformation[1][3] = _trans_y;

m_transformation[2][0] = (-1)*sin(_rot_y);
m_transformation[2][1] = cos(_rot_y)*sin(_rot_x);
m_transformation[2][2] = cos(_rot_y) * cos(_rot_x);
m_transformation[2][3] = _trans_z;

m_transformation[3][0] = 0;
m_transformation[3][1] = 0;
m_transformation[3][2] = 0;
m_transformation[3][3] = 1;
}



/* @requires: unsigned int 0<=row <4  && unsigend int 0<= coloum <4
 * @ensures: double element of 4x4 Matrix
 * */
double TMatrix::get_element(unsigned int row,  unsigned int column)
{
//returns the element on position
return m_transformation[row][column];
}


/* @requires: this TMatrix Object itself
 * @ensures:  void terminal output with std::cout
 * */
void TMatrix::output() {
    printf("Array contents: \n");

    for (int h = 0; h < 4; h++)
    {
        for (int w = 0; w < 4; w++)
        {
            printf("%f,", this->m_transformation[h][w]);
        }
        printf("\n");
    }
}


/* @requires: this TMatrix Object && TMatrix&
 * @ensures:  4x4 * 4x4 (Matrix Matrix Multiplication)
 * */
TMatrix TMatrix::operator*(const TMatrix &mat1) {

    TMatrix result;
    int a = ConfigProvider::getInstance().getParamA();
    for(int j =0; j<4; j++){                        // j is height of matrix
        for(int i =0; i< 4; i++)                    //i is width of matrix
        {
            double temp = 0.0;
            for(int k =0; k<4; k++)
            {
                temp += this->m_transformation[j][k] * mat1.m_transformation[k][i];
            }
            result.m_transformation[j][i] = temp;
        }
    }

    return result;
}

/* @requires: this TMatrix Object && std::array<double,4>&
 * @ensures:  4x4 * 4x1 (Matrix Vector Multiplication)
 * */

std::array<double, 4> TMatrix::operator*(const std::array<double, 4> &arr) {

    std::array<double, 4> result;

    for (int row = 0; row < 4; row++) {
        double sum = 0;
        for (int column = 0; column < 4; column++) {

            sum += this->m_transformation[row][column] * arr[column];
        }
        result[row] = sum;
    }

    return result;
}

/* @requires: this TMatrix TranslationMatrix 4x4 of denavit Hartenberg
 * @ensures:  double array[3] with phi, theta, psi in degree
 * */
std::array<double, 3> TMatrix::convertToEulerAngles() {
    // initialize the euler angles
    double phi, theta, psi;
    std::array<double, 3> a;
    double u = 0.0001;
    double trans_0_0 = this->m_transformation[0][0];
    double trans_0_1 = this->m_transformation[0][1];
    double trans_1_0 = this->m_transformation[1][0];
    double trans_2_0 = this->m_transformation[2][0];
    double trans_2_1 = this->m_transformation[2][1];
    double trans_2_2 = this->m_transformation[2][2];

    bool trans_20_is_0 = false;

    if(trans_0_0 <= 0.000001 && trans_0_0 >= -0.000001){
        trans_0_0 = 0;
    }

    else if(trans_0_1 <= 0.000001 && trans_0_1 >= -0.000001){
        trans_0_1 = 0;
    }

    else if(trans_1_0 <= 0.000001 && trans_1_0 >= -0.000001){
        trans_1_0 = 0;
    }

    else if(trans_2_0 <= 0.000001 && trans_2_0 >= -0.000001){
        trans_2_0 = 0;
        trans_20_is_0 = true;
    }

    else if(trans_2_1 <= 0.000001 && trans_2_1 >= -0.000001){
        trans_2_1 = 0;
    }

    else if(trans_2_2 <= 0.000001 && trans_2_2 >= -0.000001){
        trans_2_2 = 0;
    }
    // Error case
    if ((trans_0_0 <= 0 + u && trans_0_0 >= 0 - u) && (trans_1_0 <= 0 + u && trans_1_0 >= 0 - u)) {
        phi = asin( (-1)* trans_0_1);
        theta = - trans_2_0 * M_PI/2;
        psi = 0;
        std::cout << "Errorcase for Euler Angles." << std::endl;
    }
    else {  // normal case
        phi = atan2(trans_1_0, trans_0_0);
        if(trans_20_is_0 = true){
            theta = atan2(trans_2_0, sqrt( trans_2_1*trans_2_1 + trans_2_2*trans_2_2 ));
        }
        else{
            theta = atan2((-1)*trans_2_0, sqrt( trans_2_1*trans_2_1 + trans_2_2*trans_2_2 ));
        }
        //theta = atan2((-1)* trans_2_0, sqrt( trans_2_1*trans_2_1 + trans_2_2*trans_2_2 ));
        psi = atan2(trans_2_1, trans_2_2);
    }
    // add values to array
    a[0] = phi;
    a[1] = theta*(-1);
    a[2] = psi;
    for (int i = 0; i < 3; ++i) {
        if(a[i] < 0.000000001 && a[i] > -0.000000001){
            std::cout << "angle before rounding: " << a[i] << std::endl;
            a[i] = 0;
            std::cout << "rounding of euler angle: " << i << std::endl;
        }
    }
    return a;
}
