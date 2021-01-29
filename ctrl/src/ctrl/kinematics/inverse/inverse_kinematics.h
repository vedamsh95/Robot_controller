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
    const double d = .215; //215 mm  in-line wrist length
    const double m = .330; //330 mm  offset length from Zo axis
    const double n = .645;  //645 mm  offset height from base
    const double o = .115;  // 115 mm  offset of arm on linkarm
    const double a = 1.150;  // 1150 mm  linkarm 
    const double b = 1.220;  //1220 mm    arm


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
    void find_phi_123(double Xc, double Yc, double Zc, double phi1, std::vector<double> (&point1), std::vector<double> (&point2), std::vector<double> (&point3));
    void findphi4phi5phi6(double phi1, double phi2, double phi3, std::vector<double> &phi4, std::vector<double> &phi5, std::vector<double> &phi6, double transformationmatrix[16]);
    std::tuple<double, double> ForwardsElbowdown(double d1, double Zc);
    std::tuple<double, double> ForwardsElbowup(double d1, double Zc);
    std::tuple<double, double> BackwardsElbowdown(double d1, double Zc);
    std::tuple<double, double> BackwardsElbowup(double d1, double Zc);
    void multiplymatrix(double M1[4][4], double M2[4][4], double(&(Result)));
    void multiplymatrix(double M1[3][3], double M2[3][3], double(&(Result)));
    void printmatrix(double matrix[4][4]);
    void printmatrix(double matrix[3][3]);
    void degreetoradian(double & angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6);
    void radiantodegrees(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6);

};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
