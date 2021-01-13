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

std::vector<std::vector<double>> inv_standardcase(double phi1, double d1, std::array<double, 3> wcp);
std::array<double, 4> inv_forwardcase(double dpx, double dpy);
std::array<double, 4> inv_backwardcase(double dpx, double dpy);
std::vector<std::vector<double>> inv_checklimits_theta1_2_3(double phi1, array<double, 4>phi2_phi3);
std::vector<std::vector<double>> inv_checktheta(double phi1, double d1, std::array<double, 3> wcp);
std::array<double, 10> inv_gettheta4_5_6(TMatrix R36);
std::vector<Configuration*>* inv_add_case_to_vec(double phi1, double d1, std::array<double, 3> wcp, SixDPos* _pos, vector<vector<double>> solutions_vec);
std::vector<std::vector<double>> inv_vec_sol_theta4_5_6(TMatrix T03, SixDPos* _pos);
std::vector<std::vector<double>> inv_checklimits_theta4_5_6(std::array<double, 10> solution_standard_4_5_6);
};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H


