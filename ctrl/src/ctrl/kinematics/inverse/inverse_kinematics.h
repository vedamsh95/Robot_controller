#ifndef SDRI_CTRL2019_INVERSE_KINEMATICS_H
#define SDRI_CTRL2019_INVERSE_KINEMATICS_H


#include <SixDPos.h>
#include <TMatrix.h>
#include <Configuration.h>
#include "ConfigProvider.h"

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

    //Final Configuration Vector that is returned by the inverse Kinematics
    std::vector<Configuration*>* solution_vec = new vector<Configuration*>();

    //Robot Dimensnions
    double o = ConfigProvider::getInstance().geto();
    double m = ConfigProvider::getInstance().getm();
    double n = ConfigProvider::getInstance().getn();
    double a = ConfigProvider::getInstance().geta();
    double b = ConfigProvider::getInstance().getb();

    //Robot joint boundaries
    double theta1_u = ConfigProvider::getInstance().getTheta1_upper_border();
    double theta1_l = ConfigProvider::getInstance().getTheta1_lower_border();
    double theta2_u = ConfigProvider::getInstance().getTheta2_upper_border();
    double theta2_l = ConfigProvider::getInstance().getTheta2_lower_border();
    double theta3_u = ConfigProvider::getInstance().getTheta3_upper_border();
    double theta3_l = ConfigProvider::getInstance().getTheta3_lower_border();
    double theta4_u = ConfigProvider::getInstance().getTheta4_upper_border();
    double theta4_l = ConfigProvider::getInstance().getTheta4_lower_border();
    double theta5_u = ConfigProvider::getInstance().getTheta5_upper_border();
    double theta5_l = ConfigProvider::getInstance().getTheta5_lower_border();
    double theta6_u = ConfigProvider::getInstance().getTheta6_upper_border();
    double theta6_l = ConfigProvider::getInstance().getTheta6_lower_border();

    std::vector<vector<double>> solution;
    std::array<double, 4> theta2_theta3;
    std::vector<std::vector<double>> solution_standard;
    std::vector<std::vector<double>> sol_standard;
    std::array<double,10> theta4_5_6;

    std::vector<std::vector<double>> sol_specialcase1_1;
    std::vector<std::vector<double>> sol_specialcase1_2;

    //Configuration Vector that contains all possible configurations for the theta1 special case
    std::vector<Configuration*>* sol_specialcase1_1_vec = new vector<Configuration*>();
    std::vector<Configuration*>* sol_specialcase1_2_vec = new vector<Configuration*>();
    std::vector<Configuration*>* sol_specialcase1_3_vec = new vector<Configuration*>();

    //Vector with possible
    std::vector<vector<double>> sol_theta1_special1_1;
    std::vector<vector<double>> sol_theta1_special1_2;
    std::vector<vector<double>> sol_theta1_special1_3;

    // 2 vectors that are used in the othercase functions
    vector<vector<double>> sol_othercase_1_vec;
    vector<vector<double>> sol_othercase_2_vec;

    //vectors are used to store solution from othercase functions in main function
    vector<Configuration *> *sol_othercase_1_vec_config = new vector<Configuration*>();
    vector<Configuration *> *sol_othercase_2_vec_config = new vector<Configuration*>();
    vector<Configuration*>* solution_othercase_1_vec = new vector<Configuration*>();
    vector<Configuration*>* solution_othercase_2_vec = new vector<Configuration*>();

    //margin point for potential singularities
    double margin_point = ConfigProvider::getInstance().getmargin_point();

    // Functions used for inverse Kinematics
    vector<Configuration*>* get_inv_kinematics(SixDPos* _pos);

    std::vector<std::vector<double>> inv_standardcase(double theta1, double d1, std::array<double, 3> wcp);

    std::array<double, 4> inv_forwardcase(double dpx, double dpy);

    std::array<double, 4> inv_backwardcase(double dpx, double dpy);

    std::vector<std::vector<double>> inv_checklimits_theta1_2_3(double theta1, array<double, 4>theta2_theta3);

    std::vector<Configuration*>* inv_checktheta(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos);

    std::vector<Configuration*>* inv_othercase_1(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos);

    std::vector<Configuration*>* inv_othercase_2(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos);

    std::array<double, 10> inv_gettheta4_5_6(TMatrix R36);

    std::vector<Configuration*>* inv_add_case_to_vec(double theta1, double d1, std::array<double, 3> wcp, SixDPos* _pos, vector<vector<double>> solutions_vec);

    std::vector<std::vector<double>> inv_vec_sol_theta4_5_6(TMatrix R03_invert, SixDPos* _pos);

    std::vector<std::vector<double>> inv_checklimits_theta4_5_6(std::array<double, 10> solution_standard_4_5_6);

    TMatrix transposematrix(TMatrix T03);

};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H


