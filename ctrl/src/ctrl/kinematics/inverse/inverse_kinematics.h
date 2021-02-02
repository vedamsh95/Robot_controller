#ifndef SDRI_CTRL2019_INVERSE_KINEMATICS_H
#define SDRI_CTRL2019_INVERSE_KINEMATICS_H


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

    double angles_phi1(double xc,double yc, double d1,std::vector<double>& vec_phi1);
    double angles_phi2_forward(double xc,double yc);
    double angles_phi2_backward(double xc,double yc);
    double angles_phi3();
    TMatrix R36Matrix();

    void checksingularities();
};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H
