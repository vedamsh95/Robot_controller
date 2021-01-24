//
// Created by Timo on 22.01.2021.
//

#ifndef SDIR_CTRL2020_CONFIGPROVIDER_H
#define SDIR_CTRL2020_CONFIGPROVIDER_H

class ConfigProvider{
private:
    ConfigProvider(){}
    int paramA = 100; // examples
    int paramB = 200; // example
    int paramC = 300; // example
    double steps_per_second = 15;
    //ToDo: Insert paramters for boundaries, etc.

    //----------------------------------Joint-Boundaries----------------------------------------------------------------

    //Theta 1
    double theta1_lower_border = (-1)*185;
    double theta1_upper_border = 185;

    //Theta 2
    double theta2_lower_border = (-1)*120;
    double theta2_upper_border = (-1)*5;

    //Theta 3
    double theta3_lower_border = (-1)*120;
    double theta3_upper_border = 168;

    //Theta 4
    double theta4_lower_border = (-1)*350;
    double theta4_upper_border = 350;

    //Theta 5
    double theta5_lower_border = (-1)*125;
    double theta5_upper_border = 125;

    //Theta 6
    double theta6_lower_border = (-1)*350;
    double theta6_upper_border = 350;

    //----------------------------------------Axis-Boundaries------------------------------------------------------------

    //x-axis
    double x_axis_lower_border = (-1)*2701;
    double x_axis_upper_border = 2701;

    //y-axis
    double y_axis_lower_border = (-1)*2701;
    double y_axis_upper_border = 2701;

    //z-axis
    double z_axis_lower_border = (-1)*481;
    double z_axis_upper_border = 3020;

public:
    static ConfigProvider& getInstance(){
        static ConfigProvider instance;
        return instance;
    }
    ConfigProvider(ConfigProvider const&) = delete;
    void operator=(ConfigProvider const&) = delete;

    //ToDo: Insert Getter-Function for specific parameter --> Usage of ConfigProvider in specific class: ConfigProvider::getInstance().getParamA()
    int getParamA() const{return this->paramA; }
    int getParamB() const{return this->paramB; }
    int getParamC() const{return this->paramC; }
    double getsteps_per_second() const{return this->steps_per_second; }

};

#endif //SDIR_CTRL2020_CONFIGPROVIDER_H

