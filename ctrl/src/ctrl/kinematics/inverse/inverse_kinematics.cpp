#include "inverse_kinematics.h"
#include "inverse_kinematics.h"
#define _USE_MATH_DEFINES
#include "inverse_kinematics.h"
#include <math.h>
#include <tuple>
#include <TMatrix.h>
#include <vector>
#include <iostream>


//sample results for input values of (x=0.8, y = -0.3, z = 2 , e1 = 0, e2 = 90, e3 = 0

//sample values obtained
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 59.0788  phi5 : 94.2651  phi6 : 296.434
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -120.921  phi5 : -94.2651  phi6 : -243.566
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -120.921  phi5 : -94.2651  phi6 : -243.566
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -120.921  phi5 : -94.2651  phi6 : 116.434
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 239.079  phi5 : -94.2651  phi6 : -243.566
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 239.079  phi5 : -94.2651  phi6 : -243.566
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 239.079  phi5 : -94.2651  phi6 : 116.434
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 3.32495  phi5 : 113.56phi6 : -61.7082
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 3.32495  phi5 : 113.56phi6 : -61.7082
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 3.32495  phi5 : 113.56phi6 : 298.292
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -176.675  phi5 : -113.56  phi6 : -241.708
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -176.675  phi5 : -113.56  phi6 : -241.708
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : -176.675  phi5 : -113.56  phi6 : 118.292
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 183.325  phi5 : -113.56phi6 : -241.708
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 183.325  phi5 : -113.56phi6 : -241.708
//phi1 : -153.729  phi2 : -82.542  phi3 : -78.5476  phi4 : 183.325  phi5 : -113.56phi6 : 118.292


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    vector<Configuration*>* solutions = new vector<Configuration*>();

    double Xp = _pos->get_X();
    double Yp = _pos->get_Y();
    double Zp = _pos->get_Z();
    double e1 = _pos->get_A();
    double e2 = _pos->get_B();
    double e3 = _pos->get_C();

    //test 1 (x =1.75, y =2, z = 1.91, e1 =0.0, e2 = 90, e3 = 0) in GUI
    //cout<< "Printing Xp, Yp, Zp, e1, e2 and e3" << "\n";
    //cout<< Xp << "  " << Yp << "  " << Zp << "  " << e1 << "  " << e2 << "  " << e3<<"\n " ;  
    //cout<< "\n";

//converting radians into angles
    e1 = (e1 * 180) / M_PI;
    e2 = (e2 * 180) / M_PI;
    e3 = (e3 * 180) / M_PI;

    //cout << "\n Printing angles in main program\n";
    //cout << e1 <<"  "<< e2 << "   " << e3 << "\n";

//transformation matrix (in th form of an array)
    double tansformationmatrix[16];
    TMatrix Trans(e1, e2, e3, Xp, Yp, Zp);
    Trans.getTmatrix(tansformationmatrix[0]);

    //cout << "\ntransformation matrix\n";
    //for (int f = 0; f < 16; f++)
    //    cout << tansformationmatrix[f] << "  ";
    //cout << "\n";

//find WCP
    double Xc = Xp - (d * tansformationmatrix[2]);
    double Yc = Yp - (d * tansformationmatrix[6]);
    double Zc = Zp - (d * tansformationmatrix[10]);


    ////test 3
    //cout << "\nPrint wrist center point\n" << Xc << "  " << Yc << "  " << Zc << "\n";
  
//find possible values of phi1
    vector<double>temparrayphi1;
    findphi1(Xc, Yc, temparrayphi1);


    //// //test 4
    //cout<< "\nprint tempphi1array\n";
    //for(int f = 0; f < temparrayphi1.size(); f++)
    //              {
    //                 cout << temparrayphi1[f] << "\n";
    //               }


    vector<double> arrayphi1, arrayphi2, arrayphi3;
    vector<double> phi4, phi5, phi6;



//find possible conbinations of phi1, phi2, phi3

    for (int i = 0; i < temparrayphi1.size(); i++)
    {
        double phi1 = temparrayphi1[i];

        findphi2phi3(Xc, Yc, Zc, phi1, arrayphi1, arrayphi2, arrayphi3);
    }


    ////test 
    //for (int f = 0; f < arrayphi2.size(); f++)
    //{
    //    cout << "\nprinting elemnts in main program to check all combinations of phi1, phi2 and phi3\n";
    //    cout << arrayphi1[f] << "  " << arrayphi2[f] << "  " << arrayphi3[f] << "\n";
    //}

    //cout << "checking array size : \n";
    //cout << arrayphi1.size();
    //cout << arrayphi2.size();
    //cout << arrayphi3.size();
    //cout << "\n";

    
    for (int j = 0; j < arrayphi1.size(); j++)
    {
        if ((arrayphi2[j] > double(-140)) && (arrayphi2[j]< double(-5)) && (arrayphi3[j] > double(-120)) && (arrayphi3[j]<double(168)))
            {
                //
                //cout << arrayphi1[j] << "  " << arrayphi2[j] << "  " << arrayphi3[j] << "\n";
                findphi4phi5phi6(arrayphi1[j], arrayphi2[j], arrayphi3[j], phi4, phi5, phi6, tansformationmatrix);

                //cout << "testing for return values from function to satisfy the conditions\n";
                //for (int e = 0; e < phi4.size(); e++)
                //{
                //    cout << " phi1 " << arrayphi1[j] << " phi2 " << arrayphi2[j] << "phi3" << arrayphi3[j] << "phi4" << phi4[e] << "phi5" << phi5[e] << "phi6" << phi6[e]<<"\n";
                //}

                for (int r = 0; r < phi4.size(); r++)
                        {
                                    if ((phi4[r] > -350 && phi4[r] < 350) && (phi5[r] > -125 && phi5[r] < +125) && (phi6[r] > -350 && phi6[r] < 350))
                                    {
                                        cout << " phi1: " << arrayphi1[j] << "  phi2 :" << arrayphi2[j] << "  phi3  : " << arrayphi3[j] << "  phi4:  " << phi4[r] << "  phi5:  " << phi5[r] << "  phi6:  " << phi6[r]<<"\n";
                                        solutions->push_back(new Configuration({arrayphi1[j], arrayphi2[j],arrayphi3[j],phi4[r], phi5[r],phi6[r] }));
                                    }
                                    else
                                    {
                                    }

                        }
        }
        else
        {

        }
        

    }

    return solutions;
}


void InvKinematics::findphi1(double Xc, double Yc, std::vector<double>& point)
{

//changed this function from last time because backward cases was giving duplicate values
//now "findphi1" function only calculates possible forward cases of phi1, backward case is handled in the "findphi2phi3" function

    double phi1 = atan2(Yc, Xc) *(180 / M_PI);   // in angles


    if ((5 < phi1 && phi1 < 175) || ( -5 > phi1 && phi1 > -175))
    {
        if (Xc > 0 && Yc > 0) //first quadrant
        {
            point.push_back(-phi1);                //push the negative value of Phi1 into array because it is anticlockwise to +y direction
            
        }
        else if (Xc > 0 && Yc < 0)
        {
            point.push_back(-phi1);

        }
        else if (Xc < 0 && Yc>0)
        {
            point.push_back(-(phi1+180));

        }
        else if (Xc < 0 && Yc < 0)
        {
            point.push_back(180-phi1);

        }
    }

    //
    //cout << "\nPrinting value of phi1 inside findphi1 function\n" << point[0]<<"\n";
    
}

void InvKinematics::findphi2phi3(double Xc, double Yc, double Zc, double phi1, std::vector<double> (&point1), std::vector<double> (&point2), std::vector<double> (&point3))
{

    
    double d1 = sqrt((Xc * Xc) + (Yc * Yc));

    //cout <<"d1 is "<< d1<<"\n";
    //cout << "ph1 is"<< phi1 << "\n";

    double FEd2, FEd3, FEup2, FEup3, BEd2, BEd3, BEup2, BEup3;

    //find all values of possible phi2 and phi3 with the d1 and Z value
    //

    std::tie(FEd2, FEd3) = ForwardsElbowdown(d1, Zc);
    //cout << " Fed2  "<<FEd2 <<"\n"<< " Fed3 "<< FEd3<<"\n";
    //

    std::tie(FEup2, FEup3) = ForwardsElbowup(d1, Zc);
    //cout << " FEup2 "<< FEup2 << "\n" <<" FEup3 "<< FEup3 << "\n";
    //


    std::tie(BEup2, BEup3) = BackwardsElbowup(d1, Zc);
    //cout <<"BEup2"<< BEup2 << "\n" <<" BEup3"<< BEup3 << "\n";
    //


    std::tie(BEd2, BEd3) = BackwardsElbowdown(d1, Zc);
    //cout << "BEd2"<< BEd2 << "\n" << "BEd3"<< BEd3 << "\n";
    //


        if ((phi1 > 5 && phi1 < 175) || (phi1<-5 && phi1>-175))
        {
            //normal cases For. eg. 30 degrees
            // values of forward elbow down cases
            point1.push_back(phi1);                point2.push_back(FEd2);               point3.push_back(FEd3);
            point1.push_back(phi1);                point2.push_back(FEup2);              point3.push_back(FEup3);


            //possible backward cases
            //
            if (phi1 > 5)
            {
                phi1 = phi1 - 180;    //eg. -150 degrees
                point1.push_back(phi1);             point2.push_back(BEd2);   point3.push_back(BEd3);
                point1.push_back(phi1);             point2.push_back(BEup2);  point3.push_back(BEup3);

            }
            else if (phi1 < -5) // eg. -30 degrees
            {
                phi1 = phi1 + 180;     // eg. +150 degrees
                point1.push_back(phi1);             point2.push_back(BEd2);    point3.push_back(BEd3);
                point1.push_back(phi1);             point2.push_back(BEup2);   point3.push_back(BEup3);
            }

        }

        else if (phi1 < 5 && phi1 > -5)
        {

            //normal cases eg. +3 degrees
            point1.push_back(phi1);                point2.push_back(FEd2);                point3.push_back(FEd3);
            point1.push_back(phi1);                point2.push_back(FEup2);               point3.push_back(FEup3);

            //backward case 1  eg. -177 degrees
            phi1 = phi1 - 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

            //backward case 2   eg. +183 degrees
            phi1 = phi1 + 360;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

        }
        else if (phi1 < 180 && phi1 > 175)
        {
            //normal cases eg, +177 degrees  
            point1.push_back(phi1);                point2.push_back(FEd2);                point3.push_back(FEd3);
            point1.push_back(phi1);                point2.push_back(FEup2);               point3.push_back(FEup3);

            //backward case 1   eg. -3 degrees
            phi1 = phi1 - 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

            //backward case 2   eg. -183 degrees

            phi1 = phi1 - 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

        }
        else if (phi1 < -180 && phi1 < -175)
        {
            //normal cases   eg. -177 degrees
            point1.push_back(phi1);                point2.push_back(FEd2);                point3.push_back(FEd3);
            point1.push_back(phi1);                point2.push_back(FEup2);               point3.push_back(FEup3);

            //backward case 1   eg. +3 degrees 
            phi1 = phi1 + 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

            //backward case 2    eg. +183 degrees
            phi1 = phi1 - 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

        }
    
        //cout << "\nsize of phi1, 2 and 3 arrays inside the findphi2phi3 function\n" << point1.size()<<"  " << point2.size()<<"  " << point3.size()<<"\n";
        //for (int f = 0; f < point1.size(); f++)
        //{
        //    cout << "\nprinting all phi1, phi2 and phi3 inside phi1, phi2 and phi3 function\n";
        //    cout << point1[f] << "   " << point2[f] << "   " << point3[f] << "\n";
        //}

}

std::tuple<double, double> InvKinematics::ForwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt((b * b) + (o * o));

    //cout<<"print d1 and d2 inside FW Ed function"<< "\nd1  "<< d1 << "\n" <<"d2  "<< d2;
    //

    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;

        //cout << "Px" << Px<< "\n" << "  Py  " << Py;
        //

        d3 = sqrt((Px * Px) + (Py * Py));
        //beeta is b1
        //cout << "  d3\n" << d3;
        //
        b1 = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (2 * a * d2)) * (180 / M_PI);
        //alpha 1 is a1
        //cout << "   b1\n" << b1;
        a1 = asin(Py / d3) * (180 / M_PI);
        //alpha 2 is a2
        //cout << "a1\n" << a1;
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        //cout << "a2\n" << a2;
        //
        phi2 = -(a2 - a1);
        phi3 = b1 - (asin(b / d2) * (180 / M_PI)) - 90;
        /*cout << "printing sample of phi1 and phi2 in th forward function :   phi2  " << phi2<< "phi3  "<<phi3<<"\n";*/
        return make_tuple(phi2, phi3);


    }
    else if (d1 < m)
    {
        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(( (d3 * d3) - (a * a) - (d2 * d2) )/ (-2 * a * d2)) *      (180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        phi2 = 180 - (a1 - a2);
        phi3 = b1 - (asin(b / d2) * (180 / M_PI)) - 90;
        return make_tuple(phi2, phi3);
    }
    else
    {
    return make_tuple(0, 0);
    }
}

std::tuple<double, double> InvKinematics::ForwardsElbowup(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt((b * b) + (o * o));
   

    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;
        
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos( ((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) *      (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        phi2 = -(a1 + a2);
        phi3 = 360 - b1 - (asin(b/ d2) * (180 / M_PI)) - 90;
        return make_tuple(phi2, phi3);
    }

    else if (d1 < m)
    {

        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) *     (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin((sin(b1) * d2 / d3)) * (180 / M_PI);
        phi2 = 180 - (a1 + a2);
        phi3 = 360 - b1 - (asin(b / d2) * (180 / M_PI)) - 90;
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }

}

std::tuple<double, double> InvKinematics::BackwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt((b * b) + (o * o));
    if (d1 >= m || d1 == 0)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(  ((d3 * d3) - (a * a) - (d2 * d2))  / (-2 * a * d2)) *    (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        phi2 = -(180 - a1);
        phi3 = 270 - b1 - (asin(b/d2) * (180 / M_PI));
        return make_tuple(phi2, phi3);
    }
    else if (d1 < m)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(  ((d3 * d3) - (a * a )- (d2 * d2)) / (-2 * a * d2)) * (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        phi2 = -(a1 + a2);
        phi3 = 270 - b1 - (asin(b / d2) * (180 / M_PI));
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }
}

std::tuple<double, double> InvKinematics::BackwardsElbowup(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt((b * b) + (o * o));
    if (d1 >= m || d1 == 0)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) * (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin(sin(b1) * d2 / d3) * (180 / M_PI);
        phi2 = (a1 + a2) - 180;
        phi3 = -(90 - (b1 - (asin(b / d2) * (180 / M_PI))));
        return make_tuple(phi2, phi3);
    }
    else if (d1 <= m)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) * (180 / M_PI);
        a1 = asin(Py / d3) * (180 / M_PI);
        a2 = asin(sin(b1) * (d2 / d3)) * (180 / M_PI);
        phi2 = -(a1 - a2);
        phi3 = -(90 - (b1 - (asin(b / d2) * (180 / M_PI))));
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }
}

void InvKinematics::findphi4phi5phi6(double phi1, double phi2, double phi3, std::vector<double> &phi4, std::vector<double> &phi5, std::vector<double> &phi6, double tansformationmatrix[16])
{

    //find 0R3 translation about origin to offset point
    double r00[4][4];
    r00[0][0] = 1;
    r00[0][1] = 0;
    r00[0][2] = 0;
    r00[0][3] = 330;  // about x is 330mm
    r00[1][0] = 0;
    r00[1][1] = 1;
    r00[1][2] = 0;
    r00[1][3] = 0;
    r00[2][0] = 0;
    r00[2][1] = 0;
    r00[2][2] = 1;
    r00[2][3] = 645; //translation is about z?
    r00[3][0] = 0;
    r00[3][1] = 0;
    r00[3][2] = 0;
    r00[3][3] = 1;

    //cout << "\nR00\n";
    //printmatrix(r00);

    //// rotaiton about origin z axis by phi1
    double r01[4][4];
    r01[0][0] = cos(phi1);
    r01[0][1] = -sin(phi1);
    r01[0][2] = 0;
    r01[0][3] = 0;  // about x is 330mm
    r01[1][0] = sin(phi1);
    r01[1][1] = cos(phi1);
    r01[1][2] = 0;
    r01[1][3] = 0;
    r01[2][0] = 0;
    r01[2][1] = 0;
    r01[2][2] = 1;
    r01[2][3] = 0; 
    r01[3][0] = 0;
    r01[3][1] = 0;
    r01[3][2] = 0;
    r01[3][3] = 1;

    //cout << "\nR01\n";
    //printmatrix(r01);

    //matrix multiplication is product of both translation x rotation matrix 

    double netmatrix[4][4];
    multiplymatrix(r00, r01, netmatrix[0][0]);

    //cout << "\nnetmatrix is\n";
    //printmatrix(netmatrix);

    //Tmtrx about phi2 and phi3 to find the second rotational matrix
    double d2 = sqrt((o * o) + (b * b));
    double r13[4][4];
    r13[0][0] = cos(phi2 + phi3);
    r13[0][1] = sin(phi2 + phi3);
    r13[0][2] = (d2 * cos(phi2 + phi3)) + (a * cos(phi2));
    r13[0][3] = 0;
    r13[1][0] = -sin(phi2 + phi3);
    r13[1][1] = cos(phi2 + phi3);
    r13[1][2] = (d2 * sin(phi2 + phi3)) + (a * sin(phi2));
    r13[1][3] = 0;
    r13[2][0] = 0;
    r13[2][1] = 0;
    r13[2][2] = 1;
    r13[2][3] = 0;
    r13[3][0] = 0;
    r13[3][1] = 0;
    r13[3][2] = 0;
    r13[3][3] = 1;

    //net Tmatrix matrix 0R3 is product of first and second rotational matrix

    //cout << "\nR13\n";
    //printmatrix(r13);

    double R03[4][4];
    multiplymatrix(netmatrix, r13, R03[0][0]);
 
    //cout << "\nR03\n";
    //printmatrix(R03); 

//find overall transformation matrix R06 from the transformationmatrix pointer

    double T06[4][4];
    int ref = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            T06[i][j] = tansformationmatrix[ref];
            ref++;
        }

    //cout << "\nT06\n";
    //printmatrix(T06);


// multiply 0R3(Transpose)x 0R6 to be 3R6

    double R36[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R36[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                R36[i][j] += R03[k][i] * T06[k][j];
            }
        }
     
    //cout << "\nR36\n";
    //printmatrix(R36);


////now we have R36, find values of phi4, phi5 and phi6
//
////equations - 1st set
//
////*****************calculate phi5 and check for singularity conditions*******************
//

    double tempphi4 = atan2(-R36[2][1], -R36[2][0]) * (180 / M_PI);
    //cout << "\nphi4 is : "<<tempphi4<<"\n";
    while ((tempphi4 - 360) > -350) { tempphi4 -= 360; } //takes us to least possible value of tempphi4
    //cout << "\nlowest phi4 is : "<<tempphi4<<"\n";


////set of possible configuration of phi4 as per first set of equations
    int k = 0;
    while ((-350 <= tempphi4) && (tempphi4 <= 350))
    {

        double tempphi5 = atan2(sqrt(1 - ((R36[2][2]) * (R36[2][2]))), R36[2][2]) * (180 / M_PI);
       /*cout << "\nphi5 : " << tempphi5 << "\n";*/
        double tempphi6 = atan2(R36[1][2], R36[0][2]) * (180 / M_PI);
       /*cout << "\nphi6 : " << tempphi6 << "\n";*/


        while ((tempphi6 - 360) > -350) { tempphi6 -= 360; } //takes us to least possible value of tempphi6
        /*cout << "\ntempphi6 after while loop" << tempphi6 << "\n";*/
        int l = 0;
        while ((-350 < tempphi6) && (tempphi6 < 350))
        {

            phi4.push_back(tempphi4);
            phi5.push_back(tempphi5);
            phi6.push_back(tempphi6);

            /*cout << "\npossible angles : " <<tempphi4<<"\n"<< tempphi5<<"\n"<<tempphi6<<"\n";*/
            tempphi6 += (double(360*l));
            l++;
        }
        k++;
        tempphi4 += (double(360*k));
    }


////equations - 2nd set
////set of possible configuration of phi4 from second set of equations

    tempphi4 = atan2(R36[2][1], R36[2][0]) * (180 / M_PI);
    while ((tempphi4 - 360) > -350) { tempphi4 -= 360; } //takes us to least possible value of tempphi4

//    //set of possible configuration of phi4 as per first set of equations
    k = 0;
    while ((-350 < tempphi4) && (tempphi4 < 350))
    {

        double tempphi5 = atan2(-sqrt(1 - ((R36[2][2]) * (R36[2][2]))), R36[2][2]) * (180 / M_PI);
        double tempphi6 = atan2(-R36[1][2], -R36[0][2]) * (180 / M_PI);

        while ((tempphi6 - 360) > -350) { tempphi6 -= 360; }  //takes us to least possible value of tempphi6
        int l = 0;
        while ((-350 < tempphi6) && (tempphi6 < 350))
        {

            phi4.push_back(tempphi4);
            phi5.push_back(tempphi5);
            phi6.push_back(tempphi6);
            tempphi6 += (double(360)*l);
            l++;
        }
        k++;
        tempphi4 += (double(360)*k);
    }
}

void InvKinematics::multiplymatrix(double M1[4][4], double M2[4][4], double(&(Result)))
{
    //net rotational matrix 0R3 is product of first and second rotational matrix

            double* temp = &Result;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    *temp = 0;
                    for (int k = 0; k < 4; k++)
                    {
                        *temp += M1[i][k] * M2[k][j];  
                    }
                    temp++;
                }
            }
}
void InvKinematics::multiplymatrix(double M1[3][3], double M2[3][3], double(&(Result)))
{
    //net rotational matrix 0R3 is product of first and second rotational matrix

    double* temp = &Result;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
            {
                *temp = M1[i][k] * M2[k][j];
                temp++;
            }
}
void InvKinematics::printmatrix(double matrix[4][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cout << matrix[i][j] << "              ";
        }

        cout << "\n";
    }
}
void InvKinematics::printmatrix(double matrix[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << matrix[i][j] << "              ";
        }

        cout << "\n";
    }
}
