#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include "fw_kinematics.cpp"
#include <math.h>
#include<tuple>
using namespace std;

//define the Robot constants
#define d 0.215
#define m 0.330
#define n 0.645
#define o 0.115
#define a 1.150
#define b 1.220

vector<Configuration*>* InvKinematics::get_inv_kinematics(SixDPos* _pos)
{
    //TODO: IMPLEMENT Compute the inverse kinematics for a given position

    vector<Configuration*>* solutions;
    //Convert the given Position into orientation matrics
    //let Oc store the retreived Transformation matrix

    SixDPos Oc = FwKinematics::get_fw_kinematics(Configuration * _cfg);

    //let Xc, Yc and Zc be the coordinates of WCP

    double Xc = Oc[0] - d * p[13];
    double Yc = Oc[1] - d * p[14];
    double Zc = Oc[2] - d * p[15];

    //finding phi1, phi2 and phi3 are three angles
    //Multiple configurations can be possible and all should be returned as vector


    //phi1
    vector<double>arrayphi1;
    findphi1(Xc, Yc, arrayphi1);

    // find phi2 and phi3 and return to main pgm, get 1 configuation  

    for (int i = 0; i<arrayphi1.size(); i++)
    {
        int phi1 = arrayphi1[i];
        if (phi1 < -185 || phi1 > 185)
            break;
        else
        {
            vector<double> arrayphi2, arrayphi3;
            findphi2phi3(Xc, Yc, Zc, phi1, arrayphi2, arrayphi3);
            Configuration tempconfig* = new Configularion();
            int j = 0;
            for (j = 0; j < arrayphi2.size(); j++)
            {
                if (-140<arrayphi2[j]<-5 && -120<arrayphi3[j]< +168)
                {
                    tempconfig[0] = phi1;
                    tempconfig[1] = arrayphi2[j];
                    tempconfig[2] = arrayphi3[j];
                    solutions.push_back(tempconfig);
                }
                else
                {
                    break;
                }
            }
        }
    }


    //prepare the result vector for the configurations
    // you should call your inverse kinematics functions here!
    //vector<Configuration*>* solutions = new vector<Configuration*>();
    //solutions->push_back(new Configuration({0,0,1,0,0,0}));
    //solutions->push_back(new Configuration({1/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({2/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({3/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({4/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({5/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({6/8 * M_PI,0,1,0,0,0}));
    //solutions->push_back(new Configuration({7/8 * M_PI,0,1,0,0,0}));

    return solutions;


}
// PHI1
//finding arrayPhi1 with various conditions, ARRAY OF possible Phi1 angles
vector<double> findphi1(double Xc, double Yc, vector<double>& point)
{
    double phi1 = atan2(Yc, Xc) * 180 / M_PI;   // in angles

    if (0 < phi1 < 175 || 0 < phi1 < -175 || -5 < phi1 < 5)
    {
        if (Xc > 0 && Yc > 0) //first quadrant
        {
            point.push_back(-phi1);                //push the value of Phi1 into array
            point.push_back(180-phi1);           //backward case 
        }
        else if (Xc > 0 && Yc < 0)
        {
            point.push_back(-phi1);
            point.push_back(-(180 + phi1));
        }
        else if (Xc < 0 && Yc>0)
        {
            point.push_back(-phi1);
            point.push_back(180 - phi1);
        }
        else if (Xc < 0 && Yc < 0)
        {
            point.push_back(-phi1);
            point.push_back(-(180 + phi1);
        }
    }
    if (-185 < phi1 < -175)
    {
            double option1 = -(phi1);
            double option2 = -(phi1 + 180);
            double option3 = +(180 + phi1);

            point.push_back(option1);
            point.push_back(option2);
            point.push_back(option3);

    }
    else if (+175 < phi1 < +185)
    {
            double option1 = -(phi1);
            double option2 = -(phi1 + 180);
            double option3 = +(180 + phi1);

            point.push_back(option1);
            point.push_back(option2);
            point.push_back(option3);

    }
    else if (-5 < phi1 < +5)
    {
            double option1 = -(phi1);
            double option2 = -(phi1 + 180);
            double option3 = +(180 + phi1);

            point.push_back(option1);
            point.push_back(option2);
            point.push_back(option3);

    }
}



//PHI2 and PHI3
void findphi2phi3(double Xc, double Yc, double Zc, double phi1, vector<double>& point2, vector<double>& point3)
{

    double d1 = sqrt(Xc * Xc + Yc * Yc);
    double temp2, temp3;

    tie(temp2, temp3) = ForwardsElbowdown(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    tie(temp2, temp3) = ForwardsElbowup(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    tie(temp2, temp3) = BackwardsElbowup(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    tie(temp2, temp3) = BackwardsElbowdown(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
}

tuple<double, double>  ForwardsElbowdown(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);

    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        //beeta is b1
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
            //alpha 1 is a1
            a1 = asin(Py / d3) * 180 / M_PI;
            //alpha 2 is a2
            a2 = asin(sin(b1) * (d2 / d3)) * 180 / M_PI;
            phi2 = -(a2 - a1);
            phi3 = b1 - (asin(b / d2) * 180 / M_PI) - 90;
            return make_tuple(phi2, phi3);
    }
    else if (d1 < m)
    {
        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * (d2 / d3)) * 180 / M_PI;
        phi2 = 180 - (a1 - a2);
        phi3 = b1 - (asin(b / d2) * 180 / M_PI) - 90;
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }


}


tuple<double, double>  ForwardsElbowup(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;

        phi2 = -(a1 + a2);
        phi3 = 360 - b1 - (asin(b / d2) * 180 / M_PI) - 90
            return make_tuple(phi2, phi3);
    }
    else if (d1 < m)
    {

        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;

        phi2 = 180 - (a1 + a2);
        phi3 = 360 - b1 - (asin(b / d2) * 180 / M_PI) - 90;
            return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }

}

tuple<double, double>  BackwardsElbowdown(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        double Px = d1 + m;
        double Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;

        phi2 = -(180 - a1);
        phi3 = 270 - b1 - (asin(b / d2) * 180 / M_PI);
        return make_tuple(phi2, phi3);
    }
    else if (d1 < m)
    {
        double Px = d1 + m;
        double Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;

        phi2 = -(a1 + a2);
        phi3 = 270 - b1 - (asin(b / d2) * 180 / M_PI);
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }
}

tuple<double, double> BackwardsElbowup(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        double Px = d1 + m;
        double Py = Zc - n;
        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;
        phi2 = (a1 + a2) - 180;
        phi3 = -(90 - (b1 - (asin(b / d2) * 180 / M_PI));
        return make_tuple(phi2, phi3);
    }
    else if (d1 <= m)
    {
        double Px = d1 + m;
        double Py = Zc - n;
        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * 180 / M_PI);
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;
        phi2 = -(a1 - a2);
        phi3 = -(90 - (b1 - (asin(b / d2) * 180 / M_PI));
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }

}