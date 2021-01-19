#ifndef SDRI_CTRL2019_INVERSE_KINEMATICS_H
#define SDRI_CTRL2019_INVERSE_KINEMATICS_H


#include <SixDPos.h>
#include <Configuration.h>
#include<vector>


/** \brief computes the inverse kinematics
 * This class is intended to handle the computation of the inverse kinematics. This includes both the computation of the
 * configurations for the position and the orientation.
 *
 */

class InvKinematics {
private:

//define the Robot constants
    const double d = 215;
    const double m = 330;
    const double n = 645;
    const double o = 115;
    const double a = 1150;
    const double b = 1220;


public:
    /**
     * This method computes all available configurations for a given position and returns them as a vector. It is the
     * main API function for the inverse kinematics computation. It should coordinate the handling between computing the
     * configuration for the position of the wrist point and the orientation at the tcp.
     *
     * @param _pos {@ref SixDPos} to compute the configuration for
     * @return  a reference to a <code>vector<code> containing all possible configurations for the passed position
     */

    std::vector<Configuration*>* get_inv_kinematics(SixDPos* _pos);
    void findphi1(double Xc, double Yc, std::vector<double>& point);
    void findphi2phi3(double Xc, double Yc, double Zc, double phi1, std::vector<double>& point2, std::vector<double>& point3);
    void findphi3phi4phi5(double phi1, double phi2, double phi3, std::vector<double>& phi4, std::vector<double>& phi5, std::vector<double>& phi6, double* ptr2);
    std::tuple<double, double> ForwardsElbowdown(double d1, double Zc);
    std::tuple<double, double> ForwardsElbowup(double d1, double Zc);
    std::tuple<double, double> BackwardsElbowdown(double d1, double Zc);
    std::tuple<double, double> BackwardsElbowup(double d1, double Zc);

};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
