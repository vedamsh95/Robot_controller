#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <iostream>
using namespace std;
#include <TMatrix.h>


double phi1,phi2,phi3,phi4,phi5,phi6,i;
vector<double> vec_phi1,vec_phi2,vec_phi3,vec_phi4,vec_phi5,vec_phi6;
double x,y,z,r_a,r_b,r_c;
double zc ;
double m,n,a,b,o,d;
double alpha1,alpha2,beta;

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position
    m =0.330;
 n= 0.645;
 a = 1.150;
 b = 1.220;
 o = 0.115 ;
 d =0.215;


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

        double xc = x -  d * (sin(r_c) *sin(r_a) + cos(r_c) * sin(r_b) * cos(r_a));
        cout<<"value of xc "<<xc << endl;
        double yc = y - d * (-1* cos(r_c)*sin(r_a)+ sin(r_c)*sin(r_b)*cos(r_a));
    cout<<"value of yc "<<yc << endl;
        zc = z - d * (cos(r_b)*cos(r_a));

        phi1=atan(yc / (-xc))*180/M_PI;

   double d1 = sqrt(xc*xc + yc*yc);
   cout << "d1 " << d1 <<endl;


    /*for(i=0 ; i<5;i++ ) {*/
        angles_phi1(xc,yc,d1);
        angles_phi2_forward(zc,d1);
        angles_phi2_backward(zc,d1);
        angles_phi3();
        R36Matrix();


    for(i=0 ; i<vec_phi1.size();i++ ){

        if(((vec_phi1[i]>-185) &&(vec_phi1[i]<185))) {
            cout << "vec phi1 is - " << vec_phi1[i] << endl;
        }

    for(i=0 ; i<vec_phi3.size();i++ ){

      if(((vec_phi3[i]>-120) &&(vec_phi3[i]<168))) {
          cout << "vec phi3 is - " << vec_phi3[i] << endl;
      }
    }

    }
    for(i=0 ; i<vec_phi2.size();i++ ){

        if(((vec_phi2[i]>-140) &&(vec_phi2[i]<-5))) {
            cout << "vec phi2 is - " << vec_phi2[i] << endl;
        }
    }

    vector<Configuration*>* solutions = new vector<Configuration*>();
    for(i=0 ; i<vec_phi3.size();i++ ){

       if(((vec_phi1[i]>-185) &&(vec_phi1[i]<185))  && ((vec_phi3[i]>-120) &&(vec_phi3[i]<168)))  {

           cout <<" phi1 "<< vec_phi1[i]<<" phi2 "<<vec_phi2[i]<< " phi3 " << vec_phi3[i] << " phi4 " << vec_phi4[i] << " phi5 "<<vec_phi5[i] << " phi6 " << vec_phi6[i]<<  endl;
           solutions->push_back(new Configuration({vec_phi1[i],vec_phi2[i],vec_phi3[i],vec_phi4[i],vec_phi5[i],vec_phi6[i]}));
       }
    }



/*
    solutions->push_back(new Configuration({0,0,1,0,0,0}));
    solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));
*/
    return solutions;
}
//calculating  phi1 angle
 double InvKinematics::  angles_phi1(double xc,double yc,double d1) {

     if (xc < 0 && yc > 0) {
         phi1=(180 - phi1) ;
         cout << "phi1 " << phi1 <<endl;
         vec_phi1.push_back(phi1);
     } else if (xc < 0 && yc < 0) {
         phi1 = 180 - phi1;
         cout << "phi1 " << phi1 <<endl;
         vec_phi1.push_back(phi1);
     } else if (xc < 0 && yc < 0) {
         phi1 = phi1;
         cout << "phi1 " << phi1 <<endl;
         vec_phi1.push_back(phi1);
     }
     /*else if (d1 > m) {
         phi1 = -phi1;
         cout << "phi1 " << phi1 <<endl;
         vec_phi1.push_back(phi1);
     }*/
 }

 //calculating phi2 angles
 double InvKinematics::  angles_phi2_forward(double zc,double d1) {

    if(d1>m) {
        double px_dash = d1 - m;

        double py_dash = zc - n;

        double d3 = sqrt(px_dash * px_dash + py_dash * py_dash);
        cout << "d3 " << d3 << endl;
        double d2 = sqrt((o * o) + (b * b));
        cout << "d2 " << d2 << endl;
        beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) ;
        cout << "beta " << beta << endl;
        alpha2 = asin(py_dash / d3) * 180 / M_PI;
        cout << " alpha2 " << alpha2 << endl;
        alpha1 = asin(sin(beta ) * (d2 / d3)) * (180 / M_PI);
        cout << "alpha1 " << alpha1 << endl;

        double phi2_elbowdown_forward = -(alpha2 - alpha1);
        cout << "phi2 Elbowdown forward = " << phi2_elbowdown_forward << endl;
        vec_phi2.push_back(phi2_elbowdown_forward);

        double phi2_elbowup_forward = -(alpha1 + alpha2);
        cout << "phi2 Elbowdup forward = " << phi2_elbowup_forward << endl;
        vec_phi2.push_back(phi2_elbowup_forward);
    }

    }

double InvKinematics::  angles_phi2_backward(double zc,double d1) {

    if(d1<m) {
        double px_dash = m - d1;

        double py_dash = zc - n;

        double d3 = sqrt(px_dash * px_dash + py_dash * py_dash);
        cout << "d3 " << d3 << endl;
        double d2 = sqrt((o * o) + (b * b));
        cout << "d2 " << d2 << endl;
        beta = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) * 180 / M_PI;
        cout << "beta " << beta << endl;
        alpha2 = asin(py_dash / d3) * 180 / M_PI;
        cout << " alpha2 " << alpha2 << endl;
        alpha1 = asin(sin(beta * M_PI / 180) * (d2 / d3)) * (180 / M_PI);
        cout << "alpha1 " << alpha1 << endl;


        double phi2_elbowdown_backward = (alpha2 + alpha1) - 180;
        cout << "phi2 Elbowddown backward = " << phi2_elbowdown_backward << endl;
        vec_phi2.push_back(phi2_elbowdown_backward);

        double phi2_elbowup_backward = (alpha2 - alpha1) - 180;
        cout << "phi2 Elbowup backward = " << phi2_elbowup_backward << endl;
        vec_phi2.push_back(phi2_elbowup_backward);
    }
}


//calculating phi3
double InvKinematics:: angles_phi3(){

double d2 = sqrt((o * o) + (b * b));
    double phi3_elbowdown_forward = beta - asin(b/d2) -90;
    cout << "Phi3 elbowdown forward " << phi3_elbowdown_forward << endl;
    vec_phi3.push_back(phi3_elbowdown_forward);

    double phi3_elbowup_forward = 360-beta-asin(b/d2)-90;
        cout << "Phi3 elbowup forward " << phi3_elbowup_forward << endl;
        vec_phi3.push_back(phi3_elbowup_forward);

    double phi3_elbowdown_backward = 270 - beta -asin(b/d2);
        cout << "Phi3 elbowdown backward " << phi3_elbowdown_backward << endl;
        vec_phi3.push_back(phi3_elbowdown_backward);

    double phi3_elbowup_backward = -(90-(beta -asin(b/d2)));
        cout << "Phi3 elbowup backward " << phi3_elbowup_backward << endl;
        vec_phi3.push_back(phi3_elbowup_backward);
}

TMatrix InvKinematics::R36Matrix() {
    TMatrix T01(0,180,0,645);
    TMatrix T12(0+phi1,90,330,0);
    TMatrix T23(0+vec_phi2[i],0,1150,0);
    TMatrix T34(-90+vec_phi3[i],90,115,0);

    TMatrix* R03 = T01.multiply( &T12) ->multiply(&T23) -> multiply(&T34);
    R03->print();
    TMatrix* R03_T = R03->transpose();
    cout<< "Transpose of R03" << endl;
    R03_T->print();
    double bb = R03_T->get(3,3);


    TMatrix R06(x,y,z,r_a,r_b,r_c);
    cout<< "matrix of R06 " << endl;
    R06.print();

    TMatrix* R36 = R03_T->multiply(&R06);
    cout<<"R36 matrix "<<endl;
    R36->print();

    //set-1
    double phi4_1 = atan2((-1)*R36->get(1,2),(-1)*R36->get(0,2)) * 180 /M_PI;

    double phi5_1 = atan2( sqrt(1-(R36->get(2,2)*R36->get(2,2))),(-1)*R36->get(2,2))* 180 /M_PI;

    double phi6_1 = atan2(R36->get(2,1),R36->get(2,0)) * 180 /M_PI;

    //set -2
    double phi4_2 = atan2(R36->get(1,2),R36->get(0,2)) * 180 /M_PI;

    //double phi5_2 = atan2( (-1)*sqrt(1-(R36->get(3,3)*R36->get(3,3))),(-1)*R36->get(3,3))* 180 /M_PI;
    double phi5_2 = atan2( (-1)*sqrt(1-pow(R36->get(2,2),2)),(-1)*R36->get(2,2))* 180 /M_PI;

    double phi6_2 = atan2((-1)*R36->get(2,1),(-1)*R36->get(2,0)) * 180 /M_PI;

    if(-125<=phi5_1< 0) {
        if (-350 <= phi4_1 && phi4_1 < 350) {
            if (-350 <= phi6_1 && phi6_1 < 350) {

                cout << "phi4_1 " << phi4_1 << endl;
                vec_phi4.push_back(phi4_1);
                cout << "phi5_1 " << phi5_1 << endl;
                vec_phi5.push_back(phi5_1);
                cout << "phi6_1 " << phi6_1 << endl;
                vec_phi6.push_back(phi6_1);
            }
        }
    }
    if(0<phi5_2< 125){
        if (-350 <= phi4_2 && phi4_2 < 350) {
            if (-350 <= phi6_2 && phi6_2 < 350) {
        cout<< "phi4_2 " << phi4_2<< endl;
                vec_phi4.push_back(phi4_2);
        cout<< "phi5_2 " << phi5_2<< endl;
                vec_phi5.push_back(phi5_2);
        cout<< "phi6_2 " << phi6_2 << endl;
                vec_phi6.push_back(phi6_2);

            }
        }
    }
}