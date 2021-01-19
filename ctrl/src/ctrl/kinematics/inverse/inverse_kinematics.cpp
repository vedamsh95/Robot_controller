#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
using namespace std;
#include <TMatrix.h>

double phi1,phi2,phi3;
double x,y,z,r_a,r_b,r_c;
double xc,yc,zc ;
double d1,d2,d3,px_dash,py_dash;
double px,py;
double m,n,a,b,o,d;
double alpha1,alpha2,beta;
vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position
  m =330;
 n= 645;
 a = 1150;
 b = 1220;
 o = 115 ;
 d =215;


    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!

    SixDPos(x,y,z,r_a,r_b,r_c);
    x= _pos->get_X();
    cout << "value of x " << x << endl;
    y=_pos->get_Y();
    cout << "value of y " << y <<endl;
    z=_pos->get_Z();
    cout << "value of z " << z << endl;
    r_a = _pos->get_A();
    cout << " value of r_a" << r_a <<endl;
    r_b=_pos->get_B();
    cout << "value of r_b" << r_b << endl;
    r_c =_pos->get_C();
    cout << "value of r_c" << r_c << endl;

        xc = x -  d * (sin(r_c) *sin(r_a) + cos(r_c) * sin(r_b) * cos(r_a));
        cout<<"value of xc "<<xc << endl;
        yc = y - d * (-1* cos(r_c)*sin(r_a)+ sin(r_c)*sin(r_b)*cos(r_a));
    cout<<"value of yc "<<yc << endl;
        zc = z - d * (cos(r_b)*cos(r_a));

   d1 = sqrt(xc*xc + yc*yc);
   cout << "d1 " << d1 <<endl;

   angles_phi1();
   angles_phi2();
   angles_phi3();
   R36Matrix();



    vector<Configuration*>* solutions = new vector<Configuration*>();
    solutions->push_back(new Configuration({0,0,1,0,0,0}));
    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));

    return solutions;
}
//calculating  phi1 angle
 double InvKinematics::  angles_phi1() {

     if (xc < 0 && yc > 0) {
         phi1 = (180 - atan(yc / (-xc))) ;
         cout << "phi1 " << phi1 <<endl;
     } else if (xc < 0 && yc < 0) {
         phi1 = 180 - atan(yc / xc);
     } else if (xc < 0 && yc < 0) {
         phi1 = atan(yc / xc);
     } else if (d1 > m) {
         phi1 = -atan(yc / xc);
     }
 }

 //calculating phi2 angles
 double InvKinematics::  angles_phi2() {

    if(d1>m) {
        px_dash = d1 - m;

    }else if(d1<m){
        px_dash = m-d1;
    }
    py_dash = zc-n;

    d3 = sqrt(px_dash*px_dash + py_dash*py_dash);
    cout << "d3 " << d3 << endl;
    d2 = sqrt(o*o+b*b);
    cout << "d2 " << d2 << endl;
    beta = acos(((d3*d3)-(a*a)-(d2*d2))/(-2*a*d2)) ;
    cout<< "beta " << beta << endl;
    alpha2 = asin(py_dash/d3) * 180 /M_PI;
    cout << " alpha2 " << alpha2 << endl;
    alpha1 = asin(sin(beta) * (d2/d3))* 180 /M_PI ;
    cout << "alpha1 " << alpha1 << endl;

    double phi2_elbowdown_forward = -(alpha2-alpha1) ;
    cout << "phi2 Elbowdown forward = " << phi2_elbowdown_forward << endl;
    double phi2_elbowup_forward = -(alpha1+alpha2);
    cout << "phi2 Elbowdup forward = " << phi2_elbowup_forward << endl;
    double phi2_elbowdown_backward = (alpha1+alpha2) -180;
    double phi2_elbowup_backward = (alpha2-alpha1)-180;

    }

    //calculating phi3
double InvKinematics:: angles_phi3(){

    double phi3_elbowdown_forward = beta - asin(b/d2) -90;
    cout << "Phi3 elbowdown forward " << phi3_elbowdown_forward << endl;
    double phi3_elbowup_forward = 360-beta-asin(b/d2)-90;
    double phi2_elbowdown_backward = 270 - beta -asin(b/d2);
    double phi2_elbowup_backward = -(90-(beta -asin(b/d2)));

}

TMatrix InvKinematics::R36Matrix() {
    TMatrix T01(0,180,0,645);
    TMatrix T12(0+phi1,90,330,0);
    TMatrix T23(0+phi2,0,1150,0);
    TMatrix T34(-90+phi3,90,115,0);

    TMatrix* R03 = T01.multiply( &T12) ->multiply(&T23) -> multiply(&T34);
    R03->print();
    TMatrix* R03_T = R03->transpose();
    cout<< "Transpose of R03" << endl;
    R03_T->print();
    double bb = R03_T->get(3,3);
  cout << " value " << bb << endl;

    TMatrix R06(x,y,z,r_a,r_b,r_c);











}