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
};
#endif //SDRI_CTRL2019_INVERSE_KINEMATICS_H

/* test-Code for inverse kinematics
 * inverse() {
 * double o = 115;
 * double m = 330;
 * double n = 645;
 * double a = 1150;
 * double b = 1220;
 * double phi1, phi2, phi3;
 *
 *
 * std::array<double, 4> dTCP;
 * dTCP[0] = 0;
 * dTCP[1] = 0;
 * dTCP[2] = -215;
 * dTCP[3] = 1;
 *
 *
 * std::array<double, 3> wcp;
 * wcp=Transformationmatrix*dTCP;
 *
 *
 *
 * std::array<double, 6> solution;
 *
 *
 * double d1=sqrt(wcp[0]*wcp[0]+wcp[1]*wcp[1]);
 * if(wcp[0]>0 && wcp[1]>0)
 *  {
 *      phi1=-atan(wcp[1]/wcp[0]);                              //calculated phi1
 *          if(d1>m && wcp[2]>n && -175<phi1<175)               //calculating all the forward cases
 *          {
 *              double dpx = d1-m;                              //calculating the x and y components of the
 *              double dpy = wcp[2]-n;                          //distnance between the second joint and wcp
 *
 *              double d3 = sqrt(dpx*dpx+dpy+dpy);              //direct distance between joint and wcp
 *              double d2 = sqrt(o*o+b*b);
 *              double beta = acos(((d3*d3)-(a*a)-(d2*d2))/(-2*a*d2));
 *              double alpha1 = asin(sinh(beta)*(d2/d3));
 *              double alpha2 = asin(dpy/d3);
 *
 *              double phi2_f_u = -1*(alpha1+alpha2);           //for forward elbow up
 *              double phi2_f_d = -1*(alpha2-alpha1);           //for forward elbow down
 *
 *              double phi3_f_u = 360-beta-asin(b/d2)-90;       //for forward elbow up
 *              double phi3_f_d = beta-asin(b/d2)-90;           //for forward elbow down
 *
 *              if(-185< phi1 < 185)
 *              {
 *                  if(-140<phi2_f_u<-5)
 *                  {
 *                      if(-120<phi3_f_u<168)
 *                      {
 *                          solution[0]=phi1;
 *                          solution[1]=phi2_f_u;
 *                          solution[2]=phi3_f_u;
 *                      }
 *
 *                  if(-140<phi2_f_d<-5)
 *                  {
 *                      if(-120<phi3_f_d<168)
 *                      {
 *                      solution[0]=phi1;
 *                      solution[1]=phi2_f_d;
 *                      solution[2]=phi3_f_d;
 *                      }
 *                  }
 *              }
 *
 *          }
 *
 *           if(d1<m && -175<phi1<175)                              //calculating all the backward cases
 *          {
 *              double dpx = m-d1;                                  //calculating the x and y components of the
 *              double dpy = wcp[2]-n;                              //distnance between the second joint and wcp
 *
 *              double d3 = sqrt(dpx*dpx+dpy+dpy);                  //direct distance between joint and wcp
 *              double d2 = sqrt(o*o+b*b);
 *              double beta = acos(((d3*d3)-(a*a)-(d2*d2))/(-2*a*d2));
 *              double alpha1 = asin(sinh(beta)*(d2/d3));
 *              double alpha2 = asin(dpy/d3);
 *
 *              double phi2_b_u = -1*(180-(alpha1+alpha2);          //for backwards elbow up
 *              double phi2_b_d = -1*(180-(alpha2-alpha1);          //for backwards elbow down
 *
 *              double phi3_b_u = 270-beta-asin(b/d2);              //for backwards elbow up
 *              double phi3_b_d = -1*(90-(beta-asin(b/d2)));        //for backwards elbow down
 *
 *              if(-185< phi1 < 185)
 *              {
 *                  if(-140<phi2_b_u<-5)
 *                  {
 *                      if(-120<phi3_b_u<168)
 *                      {
 *                          solution[0]=phi1;
 *                          solution[1]=phi2_b_u;
 *                          solution[2]=phi3_b_u;
 *                      }
 *                  }
 *                  if(-140<phi2_b_d<-5)
 *                  {
 *                      if(-120<phi3_b_d<168)
 *                      {
 *                      solution[0]=phi1;
 *                      solution[1]=phi2_b_d;
 *                      solution[2]=phi3_b_d;
 *                      }
 *                  }
 *              }
 *  }
 *  else if(wcp[0]<0 && wcp[1]<0)
 *  {
 *      phi1=180-atan(wcp[1]/wcp[0]);
 *  }
 *  else if(wcp[0]>0 && wcp[1]<0)
 *  {
 *      phi1=atan(wcp[1]/wcp[0]);
 *  }
 *  else if(wcp[0]<0 && wcp[1]>0)
 *  {
 *      phi1=180-atan(wcp[1]/-wcp[0]);
 *  }
 *
 *  //Special cases
 *  //Special case 1:
 *  else if(wcp[0]==0 && wcp[1]>0)
 *  {
 *  }
 *  //special case 2:
 *  else if(wcp[0]==0 && wcp[1]<0)
 *  {
 *  }
 * }
 *
 */