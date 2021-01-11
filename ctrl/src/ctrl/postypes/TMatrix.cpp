#include "TMatrix.h"
#include <math.h>
#include<iostream>
#include <array>

double M_PI= 3.14;
//TODO implement your transformation type for the orientation (xyz, zyx, zyz)!
TMatrix::TMatrix() {}

TMatrix::TMatrix(double theta_n, double alpha_n, double r_n, double d_n) {
    double const fa = M_PI/180.0;


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

//Matrix is transposed!!
TMatrix::TMatrix(double _trans[6]) {
	m_transformation[0][0] = cos(_trans[3]) * cos(_trans[4]);                   //phi is first rot [3]// theta second rot [4]// psi third rot [5]
    m_transformation[1][0] = (-1)*sin(_trans[3]) * cos(_trans[5]) + cos(_trans[3]) * sin(_trans[4]) * sin(_trans[5]);
    m_transformation[2][0] = sin(_trans[3]) * sin(_trans[5]) + cos(_trans[3])*sin(_trans[4])*cos(_trans[5]);
    m_transformation[3][0] = _trans[0];


    m_transformation[0][1] = sin(_trans[3]) * cos(_trans[4]);
    m_transformation[1][1] = cos(_trans[3]) * cos(_trans[5]) + sin(_trans[3])*sin(_trans[4])*sin(_trans[5]);
    m_transformation[2][1] = (-1)*cos(_trans[3]) * sin(_trans[5]) + sin(_trans[3])*sin(_trans[4]) * sin(_trans[5]);
    m_transformation[3][1] = _trans[1];


    m_transformation[0][2] = sin(_trans[4]);
    m_transformation[1][2] = cos(_trans[4]) * sin(_trans[5]);
    m_transformation[2][2] = cos(_trans[4]) * cos(_trans[5]);
    m_transformation[3][2] = _trans[2];

    m_transformation[0][3] = 0;
    m_transformation[1][3] = 0;
    m_transformation[2][3] = 0;
    m_transformation[3][3] = 1;

}

// rotation around the z-axis     ---> phi --> rot_z
// rotation around the y-axis    ---> theta --> rot_y
// rotation around the x-axis      ---> psi -->rot_x
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




double TMatrix::get_element(unsigned int row,  unsigned int column)
{
//returns the element on position
return m_transformation[row][column];
}

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

TMatrix TMatrix::operator*(const TMatrix &mat1) {

    TMatrix result;

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



std::array<double, 3> TMatrix::convertToEulerAngles() {
    // initialize the euler angles
    double phi, theta, psi;
    std::array<double, 3> a;
    double u = 0.1;

    // Error case
    if ( (this->m_transformation[0][0] <= 0 + u || this->m_transformation[0][0] >= 0 - u)
    && (this->m_transformation[1][0] <= 0 + u || this->m_transformation[1][0] >= 0 - u)) {
        phi = asinh( - this->m_transformation[0][1] );
        theta = - this->m_transformation[2][0] * M_PI/2;
        psi = 0;
    }
    else {  // normal case
        phi = atan2(this->m_transformation[1][0], this->m_transformation[0][0]);
        theta = atan2(- this->m_transformation[2][0], sqrt( this->m_transformation[2][1]*this->m_transformation[2][1]
        + this->m_transformation[2][2]*this->m_transformation[2][2] ));
        psi = atan2(this->m_transformation[2][0], this->m_transformation[2][2]);
    }
    // add values to array
    a[0] = phi;
    a[1] = theta;
    a[2] = psi;
    return a;
}
