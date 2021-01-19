#define _USE_MATH_DEFINES

#include "inverse_kinematics.h"
#include <math.h>
#include <tuple>
#include <TMatrix.h>
#include <vector>



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

    //Input of inverse kinematics is the TCP’s position (x,y,z) and orientation 
    //in terms of yaw, pitch and roll abbreviated by (e1, e2, e3)

//1.
    double Xp = _pos->get_X();
    double Yp = _pos->get_Y();
    double Zp = _pos->get_Z();
    double e1 = _pos->get_A();
    double e2 = _pos->get_B();
    double e3 = _pos->get_C();

    //transformation matrix

    TMatrix Trans(Xp, Yp, Zp, e1, e2, e3);
    double* ptr1 = Trans.get_matrix();

//save this pointer data for later use to pass on to the phi3phi4phi5 function
    double* ptr2 = Trans.get_matrix();

//take the overall transformation matrix from pointer returned from TMatrix class
//2. 

    double Tansformationmatrix[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Tansformationmatrix[i][j] = *ptr1;
            ptr1++;
        }
    }

//3. 
//find WCP


    double Xc = Xp - d * Tansformationmatrix[2][0];
    double Yc = Yp - d * Tansformationmatrix[2][1];
    double Zc = Zp - d * Tansformationmatrix[2][2];


//4. Phi1

    vector<double>arrayphi1;
    findphi1(Xc, Yc, arrayphi1);

    // find phi2 and phi3 and return to main pgm, get 1 configuation  

//5. check for value of phi1 and find the rest of angles

    for (int i = 0; i < arrayphi1.size(); i++)
    {
        double phi1 = arrayphi1[i];
        if (phi1 < double(-185) || phi1 > double(185))
            break;
        else
        {
            vector<double> arrayphi2, arrayphi3;
            findphi2phi3(Xc, Yc, Zc, phi1, arrayphi2, arrayphi3);

            for (int j = 0; j < arrayphi2.size(); j++)
            {
//6.
                if (double(-140)< arrayphi2[j] && arrayphi2[j]< double(-5) && double(-120)<arrayphi3[j] && arrayphi3[j]<double(168))
                {


//7. find phi4, phi5, phi6 corresponding to the first 3 angles

                    vector<double> phi4, phi5, phi6;
                    findphi3phi4phi5(phi1, arrayphi2[j], arrayphi3[j], phi4, phi5, phi6, ptr2);
//8.
                    for (int r = 0; r < phi4.size(); r++)
                        solutions->push_back(new Configuration({ phi1, arrayphi2[j],arrayphi3[j],phi4[r], phi5[r],phi6[r]}));
                }

                else
                {
                    break;
                }
            }
        }
    }

    return solutions;
}

//9. finding arrayPhi1 with various conditions, ARRAY OF possible Phi1 angles

void InvKinematics::findphi1(double Xc, double Yc, std::vector<double>& point)
{

    double phi1 = atan2(Yc, Xc) * 180 / M_PI;   // in angles

    if ((5 < phi1 && phi1 < 175) || ( -5 > phi1 && phi1 > -175))
    {
        if (Xc > 0 && Yc > 0) //first quadrant
        {
            point.push_back(-phi1);                //push the value of Phi1 into array
            point.push_back(180 - phi1);           //backward case 
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
            point.push_back(-(180 + phi1));
        }
    }
    if (-185 < phi1 && phi1 < -175)
    {
        double option1 = -(phi1);
        double option2 = -(phi1 + 180);
        double option3 = +(180 + phi1);

        point.push_back(option1);
        point.push_back(option2);
        point.push_back(option3);

    }
    else if (+175 < phi1 && phi1 < +185)
    {
        double option1 = -(phi1);
        double option2 = -(phi1 + 180);
        double option3 = +(180 + phi1);

        point.push_back(option1);
        point.push_back(option2);
        point.push_back(option3);

    }
    else if (-5 < phi1 && phi1 < +5)
    {
        double option1 = -(phi1);
        double option2 = -(phi1 + 180);
        double option3 = +(180 + phi1);

        point.push_back(option1);
        point.push_back(option2);
        point.push_back(option3);

    }
}

//10. PHI2 and PHI3 returns 2 values at the same time
void InvKinematics::findphi2phi3(double Xc, double Yc, double Zc, double phi1, std::vector<double>& point2, std::vector<double>& point3)
{
    double d1 = sqrt(Xc * Xc + Yc * Yc);
    double temp2, temp3;

    std::tie(temp2, temp3) = ForwardsElbowdown(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    std::tie(temp2, temp3) = ForwardsElbowup(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    std::tie(temp2, temp3) = BackwardsElbowup(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
    std::tie(temp2, temp3) = BackwardsElbowdown(d1, Zc);
    point2.push_back(temp2);
    point3.push_back(temp3);
}

std::tuple<double, double> InvKinematics::ForwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);

    if (d1 >= m || d1 == double(0))
    {
        Px = d1 - m;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        //beeta is b1
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
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
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
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
    //return make_tuple(phi2, phi3);
}

std::tuple<double, double> InvKinematics::ForwardsElbowup(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        Px = d1 - m;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;
        phi2 = -(a1 + a2);
        phi3 = 360 - b1 - (asin(b / d2) * 180 / M_PI) - 90;
        return make_tuple(phi2, phi3);
    }
    else if (d1 < m)
    {

        Px = m - d1;
        Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
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

    //return make_tuple(phi2, phi3);
}

std::tuple<double, double> InvKinematics::BackwardsElbowdown(double d1, double Zc)
{

    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        double Px = d1 + m;
        double Py = Zc - n;

        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
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
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2)) * (180 / M_PI);
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

std::tuple<double, double> InvKinematics::BackwardsElbowup(double d1, double Zc)
{
    double d3, a1, a2, b1, Px, Py, phi2, phi3;
    double d2 = sqrt(b * b + o * o);
    if (d1 >= m || d1 == 0)
    {
        double Px = d1 + m;
        double Py = Zc - n;
        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;
        phi2 = (a1 + a2) - 180;
        phi3 = -(90 - (b1 - (asin(b / d2) * 180 / M_PI)));
        return make_tuple(phi2, phi3);
    }
    else if (d1 <= m)
    {
        double Px = d1 + m;
        double Py = Zc - n;
        d3 = sqrt(Px * Px + Py * Py);
        b1 = acos((d3 * d3 - a * a - d2 * d2) / (-2 * a * d2) * (180 / M_PI));
        a1 = asin(Py / d3) * 180 / M_PI;
        a2 = asin(sin(b1) * d2 / d3) * 180 / M_PI;
        phi2 = -(a1 - a2);
        phi3 = -(90 - (b1 - (asin(b / d2) * 180 / M_PI)));
        return make_tuple(phi2, phi3);
    }
    else
    {
        return make_tuple(0, 0);
    }
}

//11. find phi3, phi4 and phi5
void InvKinematics::findphi3phi4phi5(double phi1, double phi2, double phi3, std::vector<double>& phi4, std::vector<double>& phi5, std::vector<double>& phi6, double* ptr2)
{

    //find 0R3 rotation about phi1 to find the first rotational matrix
    double r1[3][3];
    r1[0][0] = cos(phi1);
    r1[0][1] = sin(phi1);
    r1[0][2] = 0;
    r1[1][0] = -sin(phi1);
    r1[1][1] = cos(phi1);
    r1[1][2] = 0;
    r1[2][0] = 0;
    r1[2][1] = 0;
    r1[2][2] = 1;

    double d2 = sqrt((o * o) + (b * b));

    //rotation about phi2 and phi3 to find the second rotational matrix
    double r2[3][3];
    r1[0][0] = cos(phi2 + phi3);
    r1[0][1] = sin(phi2 + phi3);
    r1[0][2] = 0;
    r1[1][0] = -sin(phi2 + phi3);
    r1[1][1] = cos(phi2 + phi3);
    r1[1][2] = 0;
    r1[2][0] = (d2 * cos(phi2 + phi3)) + (a * cos(phi2));
    r1[2][1] = (d2 * sin(phi2 + phi3)) + (a * sin(phi2));
    r1[2][2] = 1;

    //net rotational matrix 0R3 is product of first and second rotational matrix

    double R03[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R03[i][j] = 0;

    //find R03 as product of R1 and R2
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
            {
                R03[i][j] += r1[i][k] * r2[k][j];
            }

    //find transformation matrix from pointer
    double Tansformationmatrix2[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Tansformationmatrix2[i][j] = *ptr2;
            ptr2++;
        }
    }

    //find overall rotaiton matrix R06 from the transformation matrix
    double R06[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R06[i][j] = Tansformationmatrix2[i][j];

        }

    // multiply 0R3(Transpose)x 0R6 to be 3R6
    double R36[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
            {
                R36[i][j] += R03[k][i] * R06[k][j];
            }
    //now we have R36, find values of phi4, phi5 and phi6
    //using Atan2 function,

//equations - 1st set
    double tempphi4 = atan2(-R36[2][1], -R36[2][0]) * (180 / M_PI);

    //set of possible configuration of phi4 as per first set of equations
    int k = 0;
    while ((-350 < tempphi4) && (tempphi4 < 350))
    {

        double tempphi5 = atan2(sqrt(1 - ((R36[2][2]) * (R36[2][2]))), R36[2][2]) * (180 / M_PI);
        double tempphi6 = atan2(R36[1][2], R36[0][2]) * (180 / M_PI);

        int l = 0;
        while ((-350 < tempphi6) && (tempphi6 < 350))
        {

            phi4.push_back(tempphi4);
            phi5.push_back(tempphi5);
            phi6.push_back(tempphi6);
            tempphi6 += (double(2*180)*l);
            l++;
        }
        k++;
        tempphi4 += (double(2*180)*k);
    }

    //equations - 2nd set

    //set of possible configuration of phi4 from second set of equations
    tempphi4 = atan2(R36[2][1], R36[2][0]) * (180 / M_PI);

    //set of possible configuration of phi4 as per first set of equations
    k = 0;
    while ((-350 < tempphi4) && (tempphi4 < 350))
    {

        double tempphi5 = atan2(-sqrt(1 - ((R36[2][2]) * (R36[2][2]))), R36[2][2]) * (180 / M_PI);
        double tempphi6 = atan2(-R36[1][2], -R36[0][2]) * (180 / M_PI);

        int l = 0;
        while ((-350 < tempphi6) && (tempphi6 < 350))
        {

            phi4.push_back(tempphi4);
            phi5.push_back(tempphi5);
            phi6.push_back(tempphi6);
            tempphi6 += (double(2*180)*l);
            l++;
        }
        k++;
        tempphi4 += (double(2*180)*k);
    }
}
