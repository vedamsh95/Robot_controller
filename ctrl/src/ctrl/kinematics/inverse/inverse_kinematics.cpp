#include "inverse_kinematics.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <tuple>
#include <TMatrix.h>
#include <vector>
#include <iostream>
#include "../direct/fw_kinematics.h"


//steps
//1.  get final TCP details from the pointer sent to get_inv_kinematics function
//2.  take the overall transformation matrix from TMatrix function
//3.  find WCP
//4.  find phi1 call phi1 function, return an array of possible phi1 values
//5.  take each values of phi1, check if it meets the criteria, call function to find phi2 and phi3
//6.  if the returned array of phi2 and phi3 within possible values w.r.t the robot configurations, proceed to find phi4,phi5 and phi6
//7.  call phi4,phi5,phi6 function and return array of possible values
//8.  push to the possible set of configuration 
//9.  function to find phi1
//10. function to find phi2 and phi3
//11. function to find phi4, phi5 and phi6


vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    vector<Configuration*>* solutions = new vector<Configuration*>();

    double Xp = _pos->get_X();
    double Yp = _pos->get_Y();
    double Zp = _pos->get_Z();
    double e1 = _pos->get_A();
    double e2 = _pos->get_B();
    double e3 = _pos->get_C();

    //cout << "\ntest 1 \n ";
    //cout<< "Printing Xp, Yp, Zp, e1, e2 and e3" << "\n";
    //cout<< Xp << "  " << Yp << "  " << Zp << "  " << e1 << "  " << e2 << "  " << e3<<"\n " ;  
    //cout<< "\n";

//converting radians into angles
    e1 = (e1 * 180) / M_PI;
    e2 = (e2 * 180) / M_PI;
    e3 = (e3 * 180) / M_PI;

    //converting meters to millimeters
    Xp = Xp * 1000;
    Yp = Yp * 1000;
    Zp = Zp * 1000;

    //cout << "\ntest 2 \n ";
    //cout << "\n Printing angles in main program\n";
    //cout <<"e1 :"<< e1 <<"  "<<"e2 :"<< e2 << "   " <<"e3 :"<< e3 << "\n";

    //transformation matrix (in th form of an array)
    double tansformationmatrix[16];
    TMatrix Trans(e1, e2, e3, Xp, Yp, Zp);
    Trans.getTmatrix(tansformationmatrix[0]);

    /*cout << "\ntest 3 \n ";
    cout << "\ntransformation matrix\n";*/

    //for (int f = 0; f < 16; f++)
    //{ 
    //    cout << tansformationmatrix[f] << "\n";
    //}

//find WCP
    double Xc = Xp - (d * tansformationmatrix[2]);
    double Yc = Yp - (d * tansformationmatrix[6]);
    double Zc = Zp - (d * tansformationmatrix[10]);

    /*   cout << "\ntest 4 \n ";
       cout << "\nPrint wrist center point\n" << Xc << "  " << Yc << "  " << Zc << "\n";*/


       //finding first possible phi1 value
       //shoulder singularity
    double tempphi1;
    if (Xc == 0.000 && Yc == 0.000)
    {
        if (Xp == 0.000 && Yp == 0.000)
        {
            //to be dealt in the LIN / SPLINE to avoid this point and take average of the previous and next points
            tempphi1 = 0.000;
        }
        else
        {
            tempphi1 = -(atan2(Yp, Xp) * (180 / M_PI));
        }
    }
    else
    {
        tempphi1 = -(atan2(Yc, Xc) * (180 / M_PI));
    }
    cout << tempphi1;

    vector<double> arrayphi1, arrayphi2, arrayphi3;
    vector<double> phi4, phi5, phi6;
    cout << "\n TEMP phi1" << tempphi1 << "\n";


    find_phi_123(Xc, Yc, Zc, tempphi1, arrayphi1, arrayphi2, arrayphi3);

    for (int l = 0; l < arrayphi1.size(); l++)
        cout << "\n:size of array 1 2 3 test " << arrayphi1[l] << " " << arrayphi2[l] << " " << arrayphi3[l]<<"\n";


    for (int j = 0; j < arrayphi1.size(); j++)
    {
        if ((arrayphi2[j] > double(-140)) && (arrayphi2[j]< double(-5)) && (arrayphi3[j] > double(-120)) && (arrayphi3[j]<double(168)))
        {
            findphi4phi5phi6(arrayphi1[j], arrayphi2[j], arrayphi3[j], phi4, phi5, phi6, tansformationmatrix, Xc, Yc, Zc);
            
            for (int r = 0; r < phi4.size(); r++)
            {
                if ((phi4[r] > -350 && phi4[r] < 350) && (phi5[r] > -125 && phi5[r] < +125) && (phi6[r] > -350 && phi6[r] < 350))
                {
                   cout << r << " : " << arrayphi1[j] << " : " << arrayphi2[j] << ":  " << arrayphi3[j] << ":  " << phi4[r] << ":  " << phi5[r] << ":  " << phi6[r] << "\n";
                   solutions->push_back(new Configuration({ arrayphi1[j] * M_PI / 180, arrayphi2[j] * M_PI / 180,arrayphi3[j] * M_PI / 180,phi4[r] * M_PI / 180, phi5[r] * M_PI / 180,phi6[r] * M_PI / 180 }));
                }
            }
        }

    }

    return solutions;
}
    

void InvKinematics::find_phi_123(double Xc, double Yc, double Zc, double phi1, std::vector<double> (&point1), std::vector<double> (&point2), std::vector<double> (&point3))
{

    double d1 = sqrt((Xc * Xc) + (Yc * Yc));
       
    //Elbow singularity
    double Px, Py, d3;
    Px = d1 - m;
    Py = Zc - n;
    d3 = sqrt((Px * Px) + (Py * Py));

    while (d3 > double(2350))
    {

        d1 = (d1 *0.99);
        Zc = (Zc *0.99);

        Px = d1 - m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));

    }

    double FEd2, FEd3, FEup2, FEup3, BEd2, BEd3, BEup2, BEup3;

    //4 possible cases
    // if point lies below Px axis then there cannot be a elbow down condition as its not physically possible for the robot
    if ((Zc - n) < 0)
    {
        FEd2 = FEd3 = BEd2 = BEd3 = 0.000;
        std::tie(FEup2, FEup3) = ForwardsElbowup(d1, Zc);
        std::tie(BEup2, BEup3) = BackwardsElbowup(d1, Zc);
        
    }
    else
    {
        std::tie(FEd2, FEd3) = ForwardsElbowdown(d1, Zc);
        std::tie(FEup2, FEup3) = ForwardsElbowup(d1, Zc);
        std::tie(BEup2, BEup3) = BackwardsElbowup(d1, Zc);
        std::tie(BEd2, BEd3) = BackwardsElbowdown(d1, Zc);
    }

    //cout << "\nFEd2, FEd3 : "<<FEd2 <<" "<< FEd3;
    //cout << "\nFEup2, FEup3 : " << FEup2 << " " << FEup3;
    //cout << "\nBEup2, BEup3: " << BEup2 << " " << BEup3;
    //cout << "\nBEd2, BEd3 : " << BEd2 << " " << BEd3;


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
                //cout << "\nFlag GGGGGGGGGG\n";
                phi1 = (phi1 + 180);     // eg. +150 degrees
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
        else if ((phi1 < -180 && phi1 < -175))
        {
            //normal cases   eg. -177 degrees
            point1.push_back(phi1);                point2.push_back(FEd2);                point3.push_back(FEd3);
            point1.push_back(phi1);                point2.push_back(FEup2);               point3.push_back(FEup3);

            //backward case 1   eg. +3 degrees 
            phi1 = phi1 + 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

            //backward case 2    eg. +183 degrees
            phi1 = phi1 + 180;
            point1.push_back(phi1);                point2.push_back(BEd2);                point3.push_back(BEd3);
            point1.push_back(phi1);                point2.push_back(BEup2);               point3.push_back(BEup3);

        }

        //cout << "\ntest 6.1 \n ";
        //cout << "\nsize of phi1, 2 and 3 arrays inside the findphi2phi3 function\n" << point1.size()<<"  " << point2.size()<<"  " << point3.size()<<"\n";
        //for (int f = 0; f < point1.size(); f++)
        //{
        //    cout << "\nprinting all phi1, phi2 and phi3 inside phi1, phi2 and phi3 function\n";
        //    cout << point1[f] << "   " << point2[f] << "   " << point3[f] << "\n";
        //}
}

std::tuple<double, double> InvKinematics::ForwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py;
    double phi2 = 0;
    double phi3=0;
    double d2 = sqrt((b * b) + (o * o));

    //cout<<"print d1 and d2 inside FW Ed function"<< "\nd1  "<< d1 << "\n" <<"d2  "<< d2;
    

    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;

        if (Zc < 0)
            return make_tuple(0, 0);

        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos((pow(d3,2) - pow(a,2) - pow(d2,2)) / (2 * a * d2)))*(180/M_PI);
        a1 = (asin(Py / d3))*(180/M_PI);
        a2 = (asin(sin(b1*M_PI/180) * (d2 / d3)))*(180/M_PI);
        phi2 = -(a1-a2);
        phi3 = b1 - ((180/M_PI)*(asin(b / d2))) - 90;  
        
    }
    else if (d1 < m)
    {
        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(( (d3 * d3) - (a * a) - (d2 * d2) )/ (-2 * a * d2)) )* (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin(sin(b1 * M_PI / 180)) * (d2 / d3)) * (180 / M_PI);
        phi2 = -(180 - (a1 + a2));
        phi3 = b1 - ((asin(b / d2)) * (180 / M_PI)) - 90;
        
    }
    else
    {
    
    }

    //cout << "\n FORWARDS ELBOW DOWN :   phi2  " << phi2 << "phi3  " << phi3 << "\n";
    //cout<<"\n d1:" << d1 << " Zc:" << Zc;
    //cout << "\nPx" << Px << "\n" << "  Py  " << Py;
    //cout << " \n d2: " << d2;
    //cout << " \n d3: " << d3;
    //cout << "   \n b1:" << b1;
    //cout << "\n a1:" << a1;
    //cout << "\na2:" << a2;
    
    return make_tuple(phi2, phi3);
}

std::tuple<double, double> InvKinematics::ForwardsElbowup(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py;
    double phi2 = 0.000;
    double phi3 = 0.000;
    double d2 = sqrt((b * b) + (o * o));
   

    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;
        
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos( ((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2))) * (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin (sin(b1 * M_PI / 180) * (d2 / d3))) * (180 / M_PI);
        phi2 = -(a1 + a2);
        phi3 = 360 - b1 - ((asin(b/ d2)) * (180 / M_PI)) - 90;
        
    }

    else if (d1 < m)
    {

        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2)) )* (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin ( sin(b1 * M_PI / 180) * (d2 / d3))) * (180 / M_PI);
        phi2 = -(180 - (a1 - a2));
        phi3 = 360 - b1 - ((asin(b / d2)) * (180 / M_PI)) - 90;
      
    }
    else
    {
        
    }

    //cout << "\n FORWARDS ELBOW UP :   phi2  " << phi2 << "phi3  " << phi3 << "\n";
    //cout << "\n d1:" << d1 << " Zc:" << Zc;
    //cout << "\nPx" << Px << "\n" << "  Py  " << Py;
    //cout << " \n d2: " << d2;
    //cout << " \n d3: " << d3;
    //cout << "   \n b1:" << b1;
    //cout << "\n a1:" << a1;
    //cout << "\na2:" << a2;

    return make_tuple(phi2, phi3);
}

std::tuple<double, double> InvKinematics::BackwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py;
    double phi2 = 0.000;
    double phi3 = 0.000;
    double d2 = sqrt((b * b) + (o * o));
    if (d1 >= m || d1 == 0)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(  ((d3 * d3) - (a * a) - (d2 * d2))  / (-2 * a * d2)) )* (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin(sin(b1 * M_PI / 180) * (d2 / d3))) * (180 / M_PI);
        //phi2 = -(a1-a2);
        phi2 = -(180-a1);
        phi3 = 270 - b1 - ((asin(b/d2)) * (180 / M_PI));
        
    }
    else if (d1 < m)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(  ((d3 * d3) - (a * a )- (d2 * d2)) / (-2 * a * d2))) * (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin(sin(b1 * M_PI / 180) * (d2 / d3))) * (180 / M_PI);
        phi2 = -(a1 - a2);
        phi3 = 270 - b1 - ((asin(b / d2)) * (180 / M_PI));
        
    }
    else
    {
        
    }

    //cout << "\n BACKWARDS ELBOW DOWN :   phi2  " << phi2 << "phi3  " << phi3 << "\n";
    //cout << "\n d1:" << d1 << " Zc:" << Zc;
    //cout << "\nPx" << Px << "\n" << "  Py  " << Py;
    //cout << " \n d2: " << d2;
    //cout << " \n d3: " << d3;
    //cout << "   \n b1:" << b1;
    //cout << "\n a1:" << a1;
    //cout << "\na2:" << a2;

    return make_tuple(phi2, phi3);
}

std::tuple<double, double> InvKinematics::BackwardsElbowup(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py;
    double phi2 = 0.000;
    double phi3 = 0.000;
    double d2 = sqrt((b * b) + (o * o));
    if (d1 >= m || d1 == 0)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2))) * (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin(sin(b1 * M_PI / 180) * d2 / d3) )* (180 / M_PI);
        phi2 = (a1 + a2)-180;
        phi3 = -(90 - (b1 - ((asin(b / d2)) * (180 / M_PI))));
        
    }
    else if (d1 <= m)
    {
        Px = d1 + m;
        Py = Zc - n;
        d3 = sqrt((Px * Px) + (Py * Py));
        b1 = (acos(((d3 * d3) - (a * a) - (d2 * d2)) / (-2 * a * d2))) * (180 / M_PI);
        a1 = (asin(Py / d3)) * (180 / M_PI);
        a2 = (asin(sin(b1 * M_PI / 180) * (d2 / d3))) * (180 / M_PI);
        phi2 = -(a1 + a2);
        phi3 = -(90 - (b1 - ((asin(b / d2)) * (180 / M_PI))));
        
    }
    else
    {
        
    }

    //cout << "\n BACKWARDS ELBOW UP:   phi2  " << phi2 << "phi3  " << phi3 << "\n";
    //cout << "\n d1:" << d1 << " Zc:" << Zc;
    //cout << "\nPx" << Px << "\n" << "  Py  " << Py;
    //cout << " \n d2: " << d2;
    //cout << " \n d3: " << d3;
    //cout << "   \n b1:" << b1;
    //cout << "\n a1:" << a1;
    //cout << "\na2:" << a2;

    return make_tuple(phi2, phi3);
}

void InvKinematics::findphi4phi5phi6(double phi1, double phi2, double phi3, std::vector<double>& phi4, std::vector<double>& phi5, std::vector<double>& phi6, double tansformationmatrix[16], double Xc, double Yc, double Zc)
{
    double d1 = sqrt((Xc * Xc) + (Yc * Yc));
    double tempphi4, tempphi5, tempphi6;

    FwKinematics fw_kinematics;
    double* M0 = new double[16];
    M0 = fw_kinematics.D_H_matrix(0, M_PI, 0, 645);

    double* M1 = new double[16];
    M1 = fw_kinematics.D_H_matrix(phi1, M_PI * 0.5, 330, 0);


    double* M0x1 = new double[16];
    M0x1 = fw_kinematics.matrix_mult(M0, M1);


    double* M2 = new double[16];
    M2 = fw_kinematics.D_H_matrix(phi2, 0, 1150, 0);

    double* M0x1x2 = new double[16];
    M0x1x2 = fw_kinematics.matrix_mult(M0x1, M2);

    double* M3 = new double[16];
    M3 = fw_kinematics.D_H_matrix((phi3 - M_PI * 0.5), M_PI * 0.5, 115, 0);

    //cout << "M3:" << endl;
    //for (int i = 0; i < 16; i++) {
    //    cout << i << "=i,    " << M3[i] << "  ";
    //    cout << endl;
    //}

    double* M0x1x2x3 = new double[16];
    M0x1x2x3 = fw_kinematics.matrix_mult(M0x1x2, M3);

    double R03[4][4];
    int ref = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            R03[i][j] = M0x1x2x3[ref];
            ref++;
        }
    //cout << "\n R06=3\n";
    //printmatrix(R03);
    //find 0R3 translation about origin to offset point
    //double r00[4][4];
    //r00[0][0] = 1;
    //r00[0][1] = 0;
    //r00[0][2] = 0;
    //r00[0][3] = 330;  // about x is 330mm
    //r00[1][0] = 0;
    //r00[1][1] = 1;
    //r00[1][2] = 0;
    //r00[1][3] = 0;
    //r00[2][0] = 0;
    //r00[2][1] = 0;
    //r00[2][2] = 1;
    //r00[2][3] = 645; //translation is about z?
    //r00[3][0] = 0;
    //r00[3][1] = 0;
    //r00[3][2] = 0;
    //r00[3][3] = 1;
    //
    //cout << "\nR00\n";
    //printmatrix(r00);
    //// rotaiton about origin z axis by phi1
    //double r01[4][4];
    //r01[0][0] = cos(phi1 * M_PI / 180);
    //r01[0][1] = -sin(phi1 * M_PI / 180);
    //r01[0][2] = 0;
    //r01[0][3] = 0;
    //r01[1][0] = sin(phi1 * M_PI / 180);
    //r01[1][1] = cos(phi1 * M_PI / 180);
    //r01[1][2] = 0;
    //r01[1][3] = 0;
    //r01[2][0] = 0;
    //r01[2][1] = 0;
    //r01[2][2] = 1;
    //r01[2][3] = 0;
    //r01[3][0] = 0;
    //r01[3][1] = 0;
    //r01[3][2] = 0;
    //r01[3][3] = 1;
    //cout << "\nR01\n";
    //printmatrix(r01);
    //matrix multiplication is product of both translation x rotation matrix 
    //double netmatrix[4][4];
    //multiplymatrix(r00, r01, netmatrix[0][0]);
    //cout << "\nnetmatrix is\n";
    //printmatrix(netmatrix);
    ////Tmtrx about phi2 and phi3 to find the second rotational matrix
    //double d2 = sqrt((o * o) + (b * b));
    //double r13[4][4];
    //r13[0][0] = cos(phi2 + phi3);
    //r13[0][1] = sin(phi2 + phi3);
    //r13[0][2] = 0;
    //r13[0][3] = (d2 * cos(phi2 + phi3)) + (a * cos(phi2));
    //r13[1][0] = -sin(phi2 + phi3);
    //r13[1][1] = cos(phi2 + phi3);
    //r13[1][2] = 0;
    //r13[1][3] = (d2 * sin(phi2 + phi3)) + (a * sin(phi2));
    //r13[2][0] = 0;
    //r13[2][1] = 0;
    //r13[2][2] = 1;
    //r13[2][3] = 0;
    //r13[3][0] = 0;
    //r13[3][1] = 0;
    //r13[3][2] = 0;
    //r13[3][3] = 1;
    //Tmtrx about phi2 and phi3 to find the second rotational matrix
    //TMATRIX CHANGED. ROTATION OF Y is CHANGED to Z axis 
    //double d2 = sqrt((o * o) + (b * b));
    //double r13[4][4];
    //r13[0][0] = cos(phi2 + phi3);
    //r13[0][1] = sin(phi2 + phi3);
    //r13[0][2] = 0;
    //r13[0][3] = (d2 * cos(phi2 + phi3)) + (a * cos(phi2));
    //r13[2][0] = -sin(phi2 + phi3);
    //r13[2][1] = cos(phi2 + phi3);
    //r13[2][2] = 0;
    //r13[2][3] = (d2 * sin(phi2 + phi3)) + (a * sin(phi2));
    //r13[1][0] = 0;
    //r13[1][1] = 0;
    //r13[1][2] = 1;
    //r13[1][3] = 0;
    //r13[3][0] = 0;
    //r13[3][1] = 0;
    //r13[3][2] = 0;
    //r13[3][3] = 1;
    //Tmtrx about phi2 and phi3 to find the second rotational matrix
    //TMATRIX CHANGED. only translation considered in the x,y,z plane area by the 2 link arm of phi2 and phi3 
    //double d2 = sqrt((o * o) + (b * b));
    //double r13[4][4];
    //r13[0][0] = 1;
    //r13[0][1] = 0;
    //r13[0][2] = 0;
    //r13[0][3] = (d1 - m) * cos(phi1 * M_PI / 180);
    //r13[1][0] = 0;
    //r13[1][1] = 1;
    //r13[1][2] = 0;
    //r13[1][3] = (d1 - m) * sin(phi1 * M_PI / 180);
    //r13[2][0] = 0;
    //r13[2][1] = 0;
    //r13[2][2] = 1;
    //r13[2][3] = (Zc - n);
    //r13[3][0] = 0;
    //r13[3][1] = 0;
    //r13[3][2] = 0;
    //r13[3][3] = 1;
    //net Tmatrix matrix 0R3 is product of first and second rotational matrix
    //cout << "\nR13\n";
    //printmatrix(r13);
    //double R03[4][4];
    //multiplymatrix(netmatrix, r13, R03[0][0]);
    //cout << "\nR03\n";
    //printmatrix(R03); 
//find overall transformation matrix R06 from the transformationmatrix pointer

    double T06[4][4];
    ref = 0;
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
    //


    if ((acos(R36[2][2]) == 0))
    {

        tempphi4 = tempphi6 = 0.5 * sin(-(R36[0][1]) ) * (180 / M_PI);
        tempphi5 = 0;

    }
    else
    {
        tempphi4 = atan2(-R36[2][1], -R36[2][0]) * (180 / M_PI);
        //cout << "\nphi4 is : " << tempphi4 << "\n";

        while ((tempphi4 - 360) >= -350) { tempphi4 -= 360; } //takes us to least possible value of tempphi4
        //cout << "\nlowest phi4 is : " << tempphi4 << "\n";


        ////set of possible configuration of phi4 as per first set of equations
        int k = 0;
        while ((-350 <= tempphi4) && (tempphi4 <= 350))
        {
                tempphi5 = atan2(sqrt((1 - (R36[2][2] * R36[2][2]))), R36[2][2]) * (180 / M_PI);
                //cout << "\nphi5 : " << tempphi5 << "\n";
                tempphi6 = atan2(R36[1][2], R36[0][2]) * (180 / M_PI);
                //cout << "\nphi6 : " << tempphi6 << "\n";


                while ((tempphi6 - 360) > -350) 
                { tempphi6 -= 360; } //takes us to least possible value of tempphi6


                //cout << "\ntempphi6 after while loop" << tempphi6 << "\n";
                int l = 0;
                    while ((-350 < tempphi6) && (tempphi6 < 350))
                    {
                        phi4.push_back(tempphi4);
                        phi5.push_back(tempphi5);
                        phi6.push_back(tempphi6);

                        //cout << "\npushed angles : " << tempphi4 << "\n" << tempphi5 << "\n" << tempphi6 << "\n";
                        l++;
                        tempphi6 += (double(360) * l);

                    }
                k++;
                tempphi4 += (double(360) * k);

        }
        ////equations - 2nd set
        ////set of possible configuration of phi4 from second set of equations

        tempphi4 = atan2(R36[2][1], R36[2][0]) * (180 / M_PI);
        while ((tempphi4 - 360) > -350) { tempphi4 -= 360; } //takes us to least possible value of tempphi4

        //    //set of possible configuration of phi4 as per first set of equations
        k = 0;
        while ((-350 < tempphi4) && (tempphi4 < 350))
        {
                    tempphi5 = atan2(sqrt((1 - (R36[2][2] * R36[2][2]))), R36[2][2]) * (180 / M_PI);
                    tempphi6 = (atan2(-R36[1][2], -R36[0][2])) * (180 / M_PI);

                    while ((tempphi6 - 360) > -350) { tempphi6 -= 360; }  //takes us to least possible value of tempphi6
                    int l = 0;

                    while ((-350 < tempphi6) && (tempphi6 < 350))
                    {

                        phi4.push_back(tempphi4);
                        phi5.push_back(tempphi5);
                        phi6.push_back(tempphi6);
                        l++;
                        tempphi6 += (double(360) * l);
                    }
               k++;
               tempphi4 += (double(360) * k);
        }
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
void InvKinematics::degreetoradian(double & angle1, double & angle2, double &angle3, double &angle4, double &angle5, double &angle6)
{
    angle1 = angle1 * (M_PI / 180);
    angle2 = angle2 * (M_PI / 180);
    angle3 = angle3 * (M_PI / 180);
    angle4 = angle4 * (M_PI / 180);
    angle5 = angle5 * (M_PI / 180);
    angle6 = angle6 * (M_PI / 180);
}
void InvKinematics::radiantodegrees(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
{
    angle1 = angle1 * (180 / M_PI);
    angle2 = angle2 * (180 / M_PI);
    angle3 = angle3 * (180 / M_PI);
    angle4 = angle4 * (180 / M_PI);
    angle5 = angle5 * (180 / M_PI);
    angle6 = angle6 * (180 / M_PI);
}

