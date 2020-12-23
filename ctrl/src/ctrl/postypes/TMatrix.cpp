#include "TMatrix.h"
#include <math.h>
#include <cassert>

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


//TMatrix::TMatrix(double _trans[6]) {
//}


TMatrix::TMatrix(double _rot_x, double _rot_y, double _rot_z, double _trans_x, double _trans_y, double _trans_z) {
    m_transformation[0][0] = cos(_rot_z)*cos(_rot_y);
    m_transformation[0][1] = -sin(_rot_z)*cos(_rot_x)+cos(_rot_z)*sin(_rot_y)*sin(_rot_x);
    m_transformation[0][2] = sin(_rot_z)*sin(_rot_x)+cos(_rot_z)*sin(_rot_y)*cos(_rot_x);
    m_transformation[0][3] = _trans_x;
    m_transformation[1][0] = sin(_rot_z)*cos(_rot_y);;
    m_transformation[1][1] = cos(_rot_z)*cos(_rot_x)+sin(_rot_z)*sin(_rot_y)*sin(_rot_x);;
    m_transformation[1][2] = -cos(_rot_z)*sin(_rot_x)+sin(_rot_z)*sin(_rot_y)*cos(_rot_x);
    m_transformation[1][3] = _trans_y;
    m_transformation[2][0] = -sin(_rot_y);
    m_transformation[2][1] = cos(_rot_y)*sin(_rot_x);
    m_transformation[2][2] = cos(_rot_y)*cos(_rot_x);
    m_transformation[2][3] = _trans_z;
    m_transformation[3][0] = 0;
    m_transformation[3][1] = 0;
    m_transformation[3][2] = 0;
    m_transformation[3][3] = 1;
}

double TMatrix::get(int row, int col) const
{
  assert(row >= 0 && row < 4);
  assert(col >= 0 && col < 4);
  return m_transformation[row][col];
}

TMatrix TMatrix::multiply(TMatrix& right)
{
  const int N = 4;

  auto& A = m_transformation;
  auto& B = right.m_transformation;
  double C[N][N];

  for(int r = 0; r < N; r++) {
    for(int c = 0; c < N; c++) {
      double sum = 0;
      for(int i = 0; i < N; i++) {
        sum += A[r][i] * B[i][c];
      }
      C[r][c] = sum;
    }
  }

  return {
          C[0][0], C[0][1], C[0][2], C[0][3],
          C[1][0], C[1][1], C[1][2], C[1][3],
          C[2][0], C[2][1], C[2][2], C[2][3],
          C[3][0], C[3][1], C[3][2], C[3][3],
  };
}

TMatrix TMatrix::transpose()
{
    auto& A = m_transformation;
    const int N = 4;
    double C[N][N];
    
    for(int r = 0; r < N; r++){
        for(int c = 0; c < N; c++){
            C[c][r] = A[r][c];
        }
    }
    
    return {
            C[0][0], C[0][1], C[0][2], C[0][3],
            C[1][0], C[1][1], C[1][2], C[1][3],
            C[2][0], C[2][1], C[2][2], C[2][3],
            C[3][0], C[3][1], C[3][2], C[3][3],
    };
    
}

std::ostream& operator<<(std::ostream &out, const TMatrix& mat)
{
  // TODO Make the output look nice -> Equal size columns
  auto& m = mat.m_transformation;
  out << "|" << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << ", " << m[0][3] << "|" << std::endl;
  out << "|" << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << ", " << m[1][3] << "|" << std::endl;
  out << "|" << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << ", " << m[2][3] << "|" << std::endl;
  out << "|" << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ", " << m[3][3] << "|" << std::endl;
  return out;
}
