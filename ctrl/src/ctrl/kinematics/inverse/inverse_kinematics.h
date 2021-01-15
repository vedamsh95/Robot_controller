#ifndef SDRI_CTRL2019_INVERSE_KINEMATICS_H
#define SDRI_CTRL2019_INVERSE_KINEMATICS_H

#include<inverse_kinematics.cpp>
#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>

/** \brief computes the inverse kinematics
 * This class is intended to handle the computation of the inverse kinematics. This includes both the computation of the
 * configurations for the position and the orientation.
 *
 */
class InvKinematics {
public:
    /**
     * This method computes all available configurations for a given position and returns them as a vector. It is the
     * main API function for the inverse kinematics computation. It should coordinate the handling between computing the
     * configuration for the position of the wrist point and the orientation at the tcp.
     *
     * @param _pos {@ref SixDPos} to compute the configuration for
     * @return  a reference to a <code>vector<code> containing all possible configurations for the passed position
     */
    vector<Configuration*>* get_inv_kinematics(SixDPos* _pos);
    vector<double> findphi1(double Xc, double Yc, vector<double>& point);
    void findphi2phi3(double Xc, double Yc, double Zc, double phi1, vector<double>& point2, vector<double>& point3);
    void findphi3phi4phi5(double phi1, double phi2, double phi3, vector<double>& phi4, vector<double>& phi5, vector<double>& phi6, double* ptr);

};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
