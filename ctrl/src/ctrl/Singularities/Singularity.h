//
// Created by Mats on 25.01.2021.
//

#ifndef SDIR_CTRL2020_SINGULARITY_H
#define SDIR_CTRL2020_SINGULARITY_H

class Singularity{

public:
    Singularity()= default;
    std::array<double, 3> wrist_singularity(double theta4, double theta5, double theta6){

        theta4 = 0;
        theta5 = 0;
        theta6 = 0;

        std::array<double, 3> singularity_wrist{};

        singularity_wrist[0] = theta4;
        singularity_wrist[1] = theta5;
        singularity_wrist[2] = theta6;

        return singularity_wrist;

    }

};

#endif //SDIR_CTRL2020_SINGULARITY_H
