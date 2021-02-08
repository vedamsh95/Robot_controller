#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>

using namespace std;

#include <TMatrix.h>


vector<double> vec_phi1, vec_phi2, vec_phi3, vec_phi4, vec_phi5, vec_phi6;
double x, y, z, r_a, r_b, r_c;
double zc;
double m, n, a, b, o, d;
double alpha1, alpha2, beta;
vector<double> forward_vec, backward_vec, solution_phi1_1, solution_phi1_2, solution_phi1_3;
vector<double> solution_limit_f, solution_limit_b;
vector<vector<double>> *standardsol_1, vec_456;
vector<double> set1, set2, set3, set4, set5, set6, set7, set8;


const double TO_RAD = M_PI / 180;

vector<Configuration *> *InvKinematics::get_inv_kinematics(SixDPos *_pos) {
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position
    m = 0.330;
    n = 0.645;
    a = 1.150;
    b = 1.220;
    o = 0.115;
    d = 0.215;
    forward_vec.clear(), backward_vec.clear(), solution_phi1_1.clear(), solution_phi1_2.clear(), solution_phi1_3.clear();
    vec_phi4.clear(), vec_phi5.clear(), vec_phi6.clear();
    solution_limit_f.clear(), solution_limit_b.clear();
    set1.clear(), set2.clear(), set3.clear(), set4.clear(), set5.clear(), set6.clear(), set7.clear(), set8.clear();

    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!

    SixDPos(x, y, z, r_a, r_b, r_c);
    x = _pos->get_X();
    cout << "value of x " << x << endl;
    y = _pos->get_Y();
    cout << "value of y " << y << endl;
    z = _pos->get_Z();
    cout << "value of z " << z << endl;
    r_a = _pos->get_A();
    cout << " value of r_a" << r_a << endl;
    r_b = _pos->get_B();
    cout << "value of r_b" << r_b << endl;
    r_c = _pos->get_C();
    cout << "value of r_c" << r_c << endl;

    double xc = x - d * (sin(r_c) * sin(r_a) + cos(r_c) * sin(r_b) * cos(r_a));
    cout << "value of xc " << xc << endl;
    double yc = y - d * (-1 * cos(r_c) * sin(r_a) + sin(r_c) * sin(r_b) * cos(r_a));
    cout << "value of yc " << yc << endl;
    zc = z - d * (cos(r_b) * cos(r_a));
    double d1 = sqrt(xc * xc + yc * yc);
    cout << "d1 " << d1 << endl;

    double phi1_standard = atan2(yc, -(xc)) * 180 / M_PI;
    cout << "phi syndard " << phi1_standard << endl;


    double phi1;
    if (xc < 0 && yc > 0) {
        phi1 = (phi1_standard - 180);
        cout << "phi1_1 " << phi1 << endl;
        standardsol_1 = standardCase(phi1, d1, m, xc, yc, zc);
        //for (int i = 0; i < standardsol_1.size(); i++) {
        //   cout << " standard solution -> " << standardsol_1[i];
        //}

    } else if (xc < 0 && yc < 0) {
        phi1 = 180 - (atan(yc / (xc))) * 180 / M_PI;
        cout << "phi1_2 " << phi1 << endl;
        standardsol_1 = standardCase(phi1, d1, m, xc, yc, zc);

    } else if (xc < 0 && yc < 0) {
        phi1 = (atan(yc / (xc))) * 180 / M_PI;
        cout << "phi1_3 " << phi1 << endl;
        standardsol_1 = standardCase(phi1, d1, m, xc, yc, zc);
        //for (int i = 0; i < standardsol_1.size(); i++) {
        //    cout << "standard solution -> " << standardsol_1[i];
        //}
    } else if (xc > 0 && yc > 0) {
        phi1 = -(atan(yc / (xc))) * 180 / M_PI;
        cout << "phi1 " << phi1 << endl;
        standardsol_1 = standardCase(phi1, d1, m, xc, yc, zc);
        //for (int i = 0; i < standardsol_1.size(); i++) {
        //    cout << "standard solution -> " << standardsol_1[i];
        //}
    } else if (xc > 0 && yc < 0) {
        phi1 = -(atan(yc / (xc))) * 180 / M_PI;
        cout << "phi1 " << phi1 << endl;
        standardsol_1 = standardCase(phi1, d1, m, xc, yc, zc);
        //for (int i = 0; i < standardsol_1.size(); i++) {
        //    cout << "standard solution -> " << standardsol_1[i];
        //}
    }


    if (xc == 0 && yc > 0) {
        specialCase1(xc, yc, zc, m, d1);
    } else if (xc == 0 && yc < 0) {
        specialCase2(xc, yc, zc, m, d1);
    }

    R36Matrix();

    vector<Configuration *> *solutions = new vector<Configuration *>();
    //  cout << " phi1 ." << vec_phi1[10];
   /* for (int x = 0; x < standardsol_1->size(); x++) {
        vector<double> sol = standardsol_1->at(x);
        if (sol.size() < 3) continue;
        solutions->push_back(
                new Configuration({sol.at(0), sol.at(1), sol.at(2), vec_phi4[0], vec_phi5[0], vec_phi6[0]}));
    }*/

   for(int x=0 ;x<vec_456.size();x++){

       solutions->push_back(
               new Configuration({standardsol_1->at(0).at(0), standardsol_1->at(0).at(1), standardsol_1->at(0).at(2), vec_456.at(x).at(0), vec_456.at(x).at(1), vec_456.at(x).at(2)}));
   }

    // solutions->push_back(new Configuration({phi1, standardsol_1.at(i), standardsol_1.at(i), 0,0,0}));
/*    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));
*/
    return solutions;
}

//calculating phi2 angles
vector<double> InvKinematics::angles_forward(double phi1, double px_dash, double py_dash) {

    double d3 = sqrt(px_dash * px_dash + py_dash * py_dash);
    cout << "d3 " << d3 << endl;
    double d2 = sqrt((o * o) + (b * b));
    cout << "d2 " << d2 << endl;
    beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
    cout << "beta " << beta << endl;
    alpha2 = asin(py_dash / d3) * 180 / M_PI;
    cout << " alpha2 " << alpha2 << endl;
    alpha1 = asin(sin(beta) * (d2 / d3)) * (180 / M_PI);
    cout << "alpha1 " << alpha1 << endl;

    double phi2_elbowdown_forward = -(alpha2 - alpha1);
    cout << "phi2 Elbowdown forward = " << phi2_elbowdown_forward << endl;
    forward_vec.push_back(phi2_elbowdown_forward);

    double phi2_elbowup_forward = -(alpha1 + alpha2);
    cout << "phi2 Elbowdup forward = " << phi2_elbowup_forward << endl;
    forward_vec.push_back(phi2_elbowup_forward);

    double phi3_elbowdown_forward = ((beta * (180 / M_PI)) - (asin(b / d2) * (180 / M_PI)) - 90);
    cout << "Phi3 elbowdown forward " << phi3_elbowdown_forward << endl;
    forward_vec.push_back(phi3_elbowdown_forward);

    double phi3_elbowup_forward = (360 - (beta * (180 / M_PI)) - (asin(b / d2) * (180 / M_PI)) - 90);
    cout << "Phi3 elbowup forward " << phi3_elbowup_forward << endl;
    forward_vec.push_back(phi3_elbowup_forward);

    for (int i = 0; i < forward_vec.size(); i++) {
        cout << "forward vector-> " << forward_vec[i];
    }

    return forward_vec;
}


vector<double> InvKinematics::angles_backward(double phi1, double px_dash, double py_dash) {

    double d3 = sqrt(px_dash * px_dash + py_dash * py_dash);
    cout << "d3 " << d3 << endl;
    double d2 = sqrt((o * o) + (b * b));
    cout << "d2 " << d2 << endl;
    beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2));
    cout << "beta " << beta << endl;
    alpha2 = asin(py_dash / d3) * 180 / M_PI;
    cout << " alpha2 " << alpha2 << endl;
    alpha1 = asin(sin(beta) * (d2 / d3)) * (180 / M_PI);
    cout << "alpha1 " << alpha1 << endl;


    double phi2_elbowdown_backward = (alpha2 + alpha1) - 180;
    cout << "phi2 Elbowddown backward = " << phi2_elbowdown_backward << endl;
    backward_vec.push_back(phi2_elbowdown_backward);

    double phi2_elbowup_backward = (alpha2 - alpha1) - 180;
    cout << "phi2 Elbowup backward = " << phi2_elbowup_backward << endl;
    backward_vec.push_back(phi2_elbowup_backward);

    double phi3_elbowdown_backward = 270 - beta * 180 / M_PI - (asin(b / d2) * 180 / M_PI);
    cout << "Phi3 elbowdown backward " << phi3_elbowdown_backward << endl;
    backward_vec.push_back(phi3_elbowdown_backward);

    double phi3_elbowup_backward = -(90 - (beta * (180 / M_PI)) - asin(b / d2) * (180 / M_PI));
    cout << "Phi3 elbowup backward " << phi3_elbowup_backward << endl;
    backward_vec.push_back(phi3_elbowup_backward);
    for (int i = 0; i < forward_vec.size(); i++) {
        cout << "backward vector-> " << backward_vec[i];
    }

    return backward_vec;

}

vector<double> InvKinematics::limits_forward(double phi1, vector<double> forward_vec, vector<double> backward_vec) {
    if (-185 < phi1 && phi1 < 185) {
        if (-140 < forward_vec[0] && forward_vec[0] < -5) {
            if (-120 < forward_vec[2] && forward_vec[2] < 168) {
                solution_limit_f.push_back(phi1);
                solution_limit_f.push_back(forward_vec[0]);
                solution_limit_f.push_back(forward_vec[2]);
            }
        }
        if (-140 < forward_vec[1] && forward_vec[1] < -5) {
            if (-120 < forward_vec[3] && forward_vec[3] < 168) {
                solution_limit_f.push_back(phi1);
                solution_limit_f.push_back(forward_vec[1]);
                solution_limit_f.push_back(forward_vec[3]);
            }
        }
    }
    return solution_limit_f;
}

vector<double> InvKinematics::limits_backward(double phi1, vector<double> forward_vec, vector<double> backward_vec) {
    if (-185 < phi1 && phi1 < 185) {
        if (-140 < backward_vec[0] && backward_vec[0] < -5) {
            if (-120 < backward_vec[2] && backward_vec[2] < 168) {
                solution_limit_b.push_back(phi1);
                solution_limit_b.push_back(backward_vec[0]);
                solution_limit_b.push_back(backward_vec[2]);
            }
        }
        if (-140 < backward_vec[1] && backward_vec[1] < -5) {
            if (-120 < backward_vec[3] && backward_vec[3] < 168) {
                solution_limit_b.push_back(phi1);
                solution_limit_b.push_back(backward_vec[1]);
                solution_limit_b.push_back(backward_vec[3]);
            }
        }
    }
    return solution_limit_b;
}


vector<double> InvKinematics::othercase_1(double phi1, double d1, double m, double n, double zc) {

    double px_dash = d1 + m;
    double py_dash = zc - n;
    phi1 = phi1 + 180;
    vector<double> solution_other1 = angles_backward(phi1, px_dash, py_dash);
    vector<double> solution_lim_other1 = limits_backward(phi1, forward_vec, backward_vec);


    return solution_lim_other1;

}


vector<double> InvKinematics::othercase_2(double phi1, double d1, double m, double n, double zc) {

    double px_dash = d1 + m;
    double py_dash = zc - n;
    phi1 = phi1 - 180;
    vector<double> solution_other2 = angles_backward(phi1, px_dash, py_dash);
    vector<double> solution_lim_other2 = limits_backward(phi1, forward_vec, backward_vec);

    return solution_lim_other2;

}

vector<vector<double>> *InvKinematics::standardCase(double phi1, double d1, double m, double xc, double yc, double zc) {


    vector<vector<double>> *solution_phi1 = new vector<vector<double>>(phi_case1(phi1, xc, yc, zc));
    if (solution_phi1->size() > 0) {
        return solution_phi1;
    }
    vector<vector<double>> *solution_phi2 = new vector<vector<double>>(phi_case2(phi1, xc, yc, zc));
    if (solution_phi2->size() > 0) {
        return solution_phi2;
    }
    vector<vector<double>> *results = new vector<vector<double>>();
    if ((d1 > m) && (-175 < phi1 && phi1 < 175)) {

        double px_dash = d1 - m;
        double py_dash = zc - n;
        vector<double> solution_stan_1 = angles_forward(phi1, px_dash, py_dash);
        vector<double> solution_lim_stan_1 = limits_forward(phi1, forward_vec, backward_vec);

        cout << "solutionstan1 " << solution_lim_stan_1.at(0);
        cout << "solutionstan1 " << solution_lim_stan_1.at(1);
        cout << "solutionstan1 " << solution_lim_stan_1.at(2);
        // cout << "solutionstan1 " << solution_lim_stan_1.at(3);
        vector<double> solution_stan_other1 = othercase_1(phi1, d1, m, n, zc);
        vector<double> solution_stan_other2 = othercase_2(phi1, d1, m, n, zc);
        results->push_back(solution_lim_stan_1);
        results->push_back(solution_stan_other1);
        results->push_back(solution_stan_other2);

        return results;//,solution_stan_other1,solution_stan_other2;


    }

    if ((d1 < m) && (-175 < phi1 && phi1 < 175)) {

        double px_dash = m - d1;
        double py_dash = zc - n;
        vector<double> solution_stan_1 = angles_forward(phi1, px_dash, py_dash);
        vector<double> solution_lim_stan_2 = limits_forward(phi1, forward_vec, backward_vec);
        vector<double> solution_stan_other1 = othercase_1(phi1, d1, m, n, zc);
        vector<double> solution_stan_other2 = othercase_2(phi1, d1, m, n, zc);
        results->push_back(solution_lim_stan_2);
        results->push_back(solution_stan_other1);
        results->push_back(solution_stan_other2);

        return results;//,solution_stan_other1,solution_stan_other2;    }



    }
}

vector<double> InvKinematics::specialCase1(double xc, double yc, double zc, double m, double d1) {

    if (yc > m) {
        d1 = yc;
        double phi1 = -90;
        double px_dash = yc - m;
        double py_dash = zc - n;
        vector<double> special_1_1 = angles_forward(phi1, px_dash, py_dash);

        double phi1_1 = 90;
        double px_dash1 = yc + m;
        vector<double> special_1_2 = angles_backward(phi1_1, px_dash1, py_dash);

    }

    if (yc < m) {
        d1 = yc;
        double phi1 = -90;
        double px_dash = m - yc;
        double py_dash = zc - n;
        vector<double> special_1_1 = angles_backward(phi1, px_dash, py_dash);

        double phi1_1 = 90;
        double px_dash1 = yc + m;
        vector<double> special_1_2 = angles_backward(phi1_1, px_dash1, py_dash);

    }
}

vector<double> InvKinematics::specialCase2(double xc, double yc, double zc, double m, double d1) {

    if (yc > m) {
        d1 = yc;
        double phi1 = 90;
        double px_dash = yc - m;
        double py_dash = zc - n;
        vector<double> special_1_1 = angles_forward(phi1, px_dash, py_dash);

        double phi1_1 = -90;
        double px_dash1 = yc + m;
        vector<double> special_1_2 = angles_backward(phi1_1, px_dash1, py_dash);

    }

    if (yc < m) {
        d1 = yc;
        double phi1 = 90;
        double px_dash = m - yc;
        double py_dash = zc - n;
        vector<double> special_1_1 = angles_backward(phi1, px_dash, py_dash);

        double phi1_1 = -90;
        double px_dash1 = yc + m;
        vector<double> special_1_2 = angles_backward(phi1_1, px_dash1, py_dash);

    }
}


vector<vector<double>> InvKinematics::phi_case1(double phi1, double xc, double yc, double zc) {
    double m = 0.330;
    double n = 0.645;
    double d1 = sqrt(xc * xc + yc * yc);
    vector<vector<double>> *results = new vector<vector<double>>();
    if (-185 < phi1 && phi1 < -175) {
        double px_dash = d1 - m;
        double py_dash = yc - n;

        if ((d1 > m) && (yc > n)) {
            solution_phi1_1 = angles_forward(phi1, px_dash, py_dash);

            double phi1_2 = phi1 + 360;
            solution_phi1_2 = angles_forward(phi1_2, px_dash, py_dash);

            double phi_3 = phi1 + 180;
            double px_dash1 = d1 + m;
            solution_phi1_3 = angles_backward(phi_3, px_dash1, py_dash);
            results->push_back(solution_phi1_1);
            results->push_back(solution_phi1_2);
            results->push_back(solution_phi1_3);

        }
        if (d1 < m) {
            double px_dash2 = m - d1;
            solution_phi1_1 = angles_backward(phi1, px_dash2, py_dash);
            double phi1_2 = phi1 + 360;
            solution_phi1_2 = angles_backward(phi1_2, px_dash2, py_dash);
            double phi1_3 = phi1 + 180;
            double px_dash3 = d1 + m;
            solution_phi1_3 = angles_backward(phi1_3, px_dash3, py_dash);

            results->push_back(solution_phi1_1);
            results->push_back(solution_phi1_2);
            results->push_back(solution_phi1_3);
        }
    } else {}

    return *results;
}


vector<vector<double>> InvKinematics::phi_case2(double phi1, double xc, double yc, double zc) {
    double m = 0.330;
    double n = 0.645;
    double d1 = sqrt(xc * xc + yc * yc);

    vector<vector<double>> *results = new vector<vector<double>>();
    if (175 < phi1 && phi1 < 185) {
        double px_dash = d1 - m;
        double py_dash = yc - n;

        if ((d1 > m) && (yc > n)) {
            solution_phi1_1 = angles_forward(phi1, px_dash, py_dash);

            double phi1_2 = phi1 - 360;
            solution_phi1_2 = angles_forward(phi1_2, px_dash, py_dash);

            double phi_3 = phi1 - 180;
            double px_dash1 = d1 + m;
            solution_phi1_3 = angles_backward(phi_3, px_dash1, py_dash);

            results->push_back(solution_phi1_1);
            results->push_back(solution_phi1_2);
            results->push_back(solution_phi1_3);
        }
        if (d1 < m) {
            double px_dash2 = m - d1;
            solution_phi1_1 = angles_backward(phi1, px_dash2, py_dash);

            double phi1_2 = phi1 - 360;
            solution_phi1_2 = angles_backward(phi1_2, px_dash2, py_dash);

            double phi1_3 = phi1 - 180;
            double px_dash3 = d1 + m;
            solution_phi1_3 = angles_backward(phi1_3, px_dash3, py_dash);
            results->push_back(solution_phi1_1);
            results->push_back(solution_phi1_2);
            results->push_back(solution_phi1_3);

        }
    } else {}

    return *results;
}

TMatrix InvKinematics::R36Matrix() {


    TMatrix T01(0, 180 * (M_PI / 180), 0, 0.645);
    cout << "matrix of t01 " << endl;
    T01.print();

    TMatrix T12(0 + standardsol_1->at(0).at(0) * (M_PI / 180), 90 * (M_PI / 180), 0.330, 0);
    cout << "matrix of t12 " << endl;
    T12.print();
    TMatrix T23(0 + standardsol_1->at(0).at(1) * (M_PI / 180), 0, 1.150, 0);
    cout << "matrix of t23" << endl;
    T23.print();
    TMatrix T34((-90 + standardsol_1->at(0).at(2)) * (M_PI / 180), 90 * (M_PI / 180), 0.115, 0);
    cout << "matrix of t34 " << endl;
    T34.print();

    TMatrix *R03 = T01.multiply((&T12))->multiply((&T23))->multiply((&T34));
    cout << "r03" << endl;
    R03->print();
    TMatrix *R03_T = R03->transpose();
    cout << "Transpose of R03" << endl;
    R03_T->print();
    double bb = R03_T->get(3, 3);
    TMatrix R06_1(r_a, r_b, r_c, x, y, z);
    cout << "R06_1" << endl;
    R06_1.print();

    /* TMatrix *R06 = (new TMatrix(
             -5.07791870e-01, 6.35025673e-01, -5.82142433e-01, -7.18376830e+02 / 1000,
             2.62242221e-01, 7.57620537e-01, 5.97695691e-01, 1.89742719e+03 / 1000,
             8.20595171e-01, 1.50842688e-01, -5.51244092e-01, 2.32490439e+02 / 1000,
             0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00
     ));
     cout << "matrix of R06 " << endl;
     R06->print();*/

    TMatrix *R36 = R03_T->multiply(&R06_1);
    cout << "R36 matrix " << endl;
    R36->print();


    double phi4_1_1, phi4_2_1, phi6_1_1, phi6_2_1;
    //set-1
    double phi4_1 = atan2((-1) * R36->get(1, 2), (-1) * R36->get(0, 2)) * 180 / M_PI;
    cout << "set phi4_1 " << phi4_1 << endl;
    double phi5_1 = atan2(sqrt(1 - (R36->get(2, 2) * R36->get(2, 2))), (-1) * R36->get(2, 2)) * 180 / M_PI;
    cout << "set phi5_1 " << phi5_1 << endl;
    double phi6_1 = atan2(R36->get(2, 1), R36->get(2, 0)) * 180 / M_PI;
    cout << "set phi6_1 " << phi6_1 << endl;

    atan2((-1) * sqrt(1 - pow(R36->get(2, 2), 2)), (-1) * R36->get(2, 2)) * 180 / M_PI;
    atan2(sqrt(1 - (R36->get(2, 2) * R36->get(2, 2))), (-1) * R36->get(2, 2)) * 180 / M_PI;

    if (phi4_1 > 0) {
        phi4_1_1 = phi4_1 - 360;
        cout << " set phi4_1_1 " << phi4_1_1 << endl;
    } else {
        phi4_1_1 = phi4_1 + 360;
        cout << " set phi4_1_2 " << phi4_1_1 << endl;
    }

    if (phi6_1 > 0) {
        phi6_1_1 = phi6_1 - 360;
        cout << "set phi6_1_1 " << phi6_1_1 << endl;
    } else {
        phi6_1_1 = phi6_1 + 360;
        cout << "set phi6_1_2 " << phi6_1_1 << endl;
    }


//set -2
    double phi4_2 = atan2(R36->get(1, 2), R36->get(0, 2)) * 180 / M_PI;
    cout << "set phi4_2 " << phi4_2 << endl;
//double phi5_2 = atan2( (-1)*sqrt(1-(R36->get(3,3)*R36->get(3,3))),(-1)*R36->get(3,3))* 180 /M_PI;
    double phi5_2 = atan2((-1) * sqrt(1 - pow(R36->get(2, 2), 2)), (-1) * R36->get(2, 2)) * 180 / M_PI;
    cout << "set phi5_2 " << phi5_2 << endl;
    double phi6_2 = atan2((-1) * R36->get(2, 1), (-1) * R36->get(2, 0)) * 180 / M_PI;
    cout << "set phi6_2 " << phi6_2 << endl;

    if (phi4_2 > 0) {
        phi4_2_1 = phi4_2 - 360;
        cout << "set phi4_2_1 " << phi4_2_1 << endl;
    } else {
        phi4_2_1 = phi4_2 + 360;
        cout << "set phi4_2_2 " << phi4_2_1 << endl;
    }

    if (phi6_2 > 0) {
        phi6_2_1 = phi6_2 - 360;
        cout << "set phi6_2_1 " << phi6_2_1 << endl;
    } else {
        phi6_2_1 = phi6_2 + 360;
        cout << "set phi6_2_2 " << phi6_2_1 << endl;
    }

    if (0 < phi5_1 && phi5_1 < 125) {
        if (-350 <= phi4_1 && phi4_1 < 350) {
            if (-350 <= phi6_1 && phi6_1 < 350) {
                set1.push_back(phi4_1);
                set1.push_back(phi5_1);
                set1.push_back(phi6_1);
                vec_456.push_back(set1);
            }
            if (-350 <= phi6_1_1 && phi6_1_1 < 350) {
                set2.push_back(phi4_1);
                set2.push_back(phi5_1);
                set2.push_back(phi6_1_1);
                vec_456.push_back(set2);
            }
        }
        if (-350 <= phi4_1_1 && phi4_1_1 < 350) {
            if (-350 <= phi6_1 && phi6_1 < 350) {
                set3.push_back(phi4_1_1);
                set3.push_back(phi5_1);
                set3.push_back(phi6_1);
                vec_456.push_back(set3);
            }
            if (-350 <= phi6_1_1 && phi6_1_1 < 350) {
                set4.push_back(phi4_1_1);
                set4.push_back(phi5_1);
                set4.push_back(phi6_1_1);
                vec_456.push_back(set4);
            }
        }
    }
        if (-125 < phi5_2 && phi5_2 < 0) {
            if (-350 <= phi4_2 && phi4_2 < 350) {
                if (-350 < phi6_2 && phi6_2 < 350) {

                    set5.push_back(phi4_2);
                    set5.push_back(phi5_2);
                    set5.push_back(phi6_2);
                    vec_456.push_back(set5);
                }
                if (-350 <= phi6_2_1 && phi6_2_1 < 350) {
                    set6.push_back(phi4_2);
                    set6.push_back(phi5_2);
                    set6.push_back(phi6_2_1);
                    vec_456.push_back(set6);
                }
            }
            if (-350 <= phi4_2_1 && phi4_2_1 < 350) {
                if (-350 <= phi6_2 && phi6_2 < 350) {
                    set7.push_back(phi4_2_1);
                    set7.push_back(phi5_2);
                    set7.push_back(phi6_2);
                    vec_456.push_back(set7);
                }
                if (-350 <= phi6_2_1 && phi6_2_1 < 350) {
                    set8.push_back(phi4_2_1);
                    set8.push_back(phi5_2);
                    set8.push_back(phi6_2_1);
                    vec_456.push_back(set8);
                }
                }
            }
        }



    void InvKinematics::checkSingularities() {
//



    }