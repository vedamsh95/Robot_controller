//
// Created by Mats on 25.01.2021.
//

#ifndef SDIR_CTRL2020_SINGULARITY_H
#define SDIR_CTRL2020_SINGULARITY_H

class Singularity{

public:
    Singularity()= default;
    std::array<double, 7> wrist_singularity(double theta4, double theta4_1, double theta4_2, double theta5, double theta6,
                                            double theta6_1, double theta6_2){

        theta4 = 0;
        theta4_1 = 360;
        theta4_2 = -360;
        theta5 = 0;
        theta6 = 0;
        theta6_1 = 360;
        theta6_2 = -360;

        std::array<double, 7> singularity_wrist{};

        singularity_wrist[0] = theta4;
        singularity_wrist[1] = theta4_1;
        singularity_wrist[2] = theta4_2;
        singularity_wrist[3] = theta5;
        singularity_wrist[4] = theta6;
        singularity_wrist[5] = theta6_1;
        singularity_wrist[6] = theta6_2;

        return singularity_wrist;

    }
bool wrist_singularity_bool(std::vector<Configuration*>* config_vec){

    for (unsigned int i = 0; i < config_vec->size(); ++i) {
        if (config_vec->at(i)->get_configuration().operator[](4) == 0){
            return true;
        }

    }
    return false;
}

bool shoulder_singularity_bool(std::vector<Configuration*>* config_vec){
    if (config_vec->size() == 1 && config_vec->at(config_vec->size()-1)->get_configuration().operator[](0) == 0){
        return true;
    }
    return false;
}


};

#endif //SDIR_CTRL2020_SINGULARITY_H
