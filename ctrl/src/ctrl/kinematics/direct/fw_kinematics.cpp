#define _USE_MATH_DEFINES

#include <math.h>
#include "fw_kinematics.h"
#include <iostream>
#include <array>




SixDPos* FwKinematics::get_fw_kinematics(Configuration *_cfg)
{
    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.
    double theta1 = _cfg->operator[](0);    //getting theta 0
    std::cout << "Theta 1 from config: " <<  theta1<< std::endl;
    double theta2 = _cfg->operator[](1);    //getting theta 1
    std::cout << "Theta 2 from config: " << theta2<< std::endl;
    double theta3 = _cfg->operator[](2);    //getting theta 2
    std::cout << "Theta 3 from config: " <<  theta3<< std::endl;
    double theta4 = _cfg->operator[](3);    //getting theta 3
    std::cout << "Theta 4 from config: " << theta4<< std::endl;
    double theta5 = _cfg->operator[](4);    //getting theta 4
    std::cout << "Theta 5 from config: " << theta5<< std::endl;
    double theta6 = _cfg->operator[](5);    //getting theta 5
    std::cout << "Theta 6 from config: " << theta6<< std::endl;

    std::array<double, 3> a{};


    TMatrix mat1(0, 180, 0, 645);                   //creation of Matrix 1 for first joint
    TMatrix mat2((0 + theta1), 90, 330, 0);         //creation of Matrix 2 for second joint
    TMatrix mat3((0 + theta2), 0, 1150, 0);         //creation of Matrix 3 for third joint
    TMatrix mat4((-90 + theta3), 90, 115, 0);       //creation of Matrix 4 for fourth joint
    TMatrix mat5((0 + theta4),-90 ,0, -1220);       //creation of Matrix 5 for fifth joint
    TMatrix mat6((0 + theta5),90 ,0, 0);            //creation of Matrix 6 for sixth joint
    TMatrix mat7((180+theta6),180, 0, -215);        //creation of Matrix 7 for seventh joint

    TMatrix res;
    res = mat1 * mat2 * mat3 * mat4 * mat5 * mat6 * mat7;                   //calculation of final translationmatrix Denavit Hartenberg

    res.output();
    a = res.convertToEulerAngles();

    std::cout << "Phi " << a[0] << std::endl;
    std::cout << "Theta " << a[1] << std::endl;
    std::cout << "Psi " << a[2] << std::endl;

    // return new SixDPos(1.757, 0.0, 1.91, 0, M_PII, 0);
     return new SixDPos(res.get_element(0,3)/1000, res.get_element(1,3)/1000, res.get_element(2,3)/1000, a[0], a[1], a[2]);
}
