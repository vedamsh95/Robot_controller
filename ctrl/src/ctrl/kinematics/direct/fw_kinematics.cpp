#define _USE_MATH_DEFINES

#include <math.h>
#include "fw_kinematics.h"


SixDPos* FwKinematics::get_fw_kinematics(Configuration *_cfg)
{
    //TODO: IMPLEMENT the computation of the forward kinematics and derive position and euler angles. Keep in mind your
    //                definition of the rotations and whether you are working in deg or rad as well as in m or cm.



    return new SixDPos(1.757, 0.0, 1.91, 0, M_PI, 0);
}





double** calculation(double **mat1, double **mat2){

    double** array = 0;
    array = new double*[4];                         //width

    for(int j =0; j<4; j++){                        // j is height of matrix
        array[j] = new double[4];

        for(int i =0; i< 4; i++)                    //i is width of matrix
        {
            double temp = 0.0;
            for(int k =0; k<4; k++)
            {
                temp += mat1[j][k] * mat2[k][i];
            }
            array[j][i] = temp;
            //std::cout << temp << " ";
        }
        //std::cout << std::endl;
    }

    return array;
}


double** create(double theta_n, double alpha_n, double r_n, double d_n){

    double const fa = M_PI/180.0;

    double** array = 0;
    array = new double*[4];

    for (int h = 0; h < 4; h++)
    {
        array[h] = new double[4];

        for (int w = 0; w < 4; w++)
        {
            if(h == 0 && w == 0){
                array[h][w] = cos(theta_n*fa);
            }
            else if (h==0 && w == 1){
                array[h][w] = -sin(theta_n*fa)*cos(alpha_n*fa);
            }
            else if (h==0 && w == 2){
                array[h][w] = sin(theta_n*fa)*sin(alpha_n*fa);
            }
            else if (h==0 && w == 3){
                array[h][w] = r_n*cos(theta_n*fa);
            }
            else if(h == 1 && w == 0){
                array[h][w] = sin(theta_n*fa);
            }
            else if (h==1 && w == 1){
                array[h][w] = cos(theta_n*fa)*cos(alpha_n*fa);
            }
            else if (h==1 && w == 2){
                array[h][w] = -cos(theta_n*fa)*sin(alpha_n*fa);
            }
            else if (h==1 && w == 3){
                array[h][w] = r_n*sin(theta_n*fa);
            }
            else if(h == 2 && w == 0){
                array[h][w] = 0;
            }
            else if (h==2 && w == 1){
                array[h][w] = sin(alpha_n*fa);
            }
            else if (h==2 && w == 2){
                array[h][w] = cos(alpha_n*fa);
            }
            else if (h==2 && w == 3){
                array[h][w] = d_n;
            }
            else if(h == 3 && w == 0){
                array[h][w] = 0;
            }
            else if (h==3 && w == 1){
                array[h][w] = 0;
            }
            else if (h==3 && w == 2){
                array[h][w] = 0;
            }
            else if (h==3 && w == 3){
                array[h][w] = 1;
            }

        }
    }

    return array;
}




int main(){
    std::cout << "Test started..."<< std::endl;

    enum theta {theta1 = 0, theta2=0, theta3 =0, theta4=0, theta5=0, theta6=0};

    double** m1 = create(0, 180, 0, 645);
    double** m2 = create((0 + theta1), 90, 330, 0);
    double** m3 = create((0 + theta2), 0, 1150, 0);
    double** m4 = create((-90 + theta3), 90, 115, 0);
    double** m5 = create((0 + theta4),-90 ,0, -1220);
    double** m6 = create((0 + theta5),90 ,0, 0);
    double** m7 = create((180+theta6),180, 0, -215);


    double** result = calculation(calculation(calculation(calculation(calculation(calculation(m1,m2),m3),m4),m5),m6),m7);

    printf("Array contents: \n");

    for (int h = 0; h < 4; h++)
    {
        for (int w = 0; w < 4; w++)
        {
            printf("%f,", result[h][w]);
        }
        printf("\n");
    }

    return 0;

}