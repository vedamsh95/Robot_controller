//
// Created by Timo on 22.01.2021.
//

#ifndef SDIR_CTRL2020_CONFIGPROVIDER_H
#define SDIR_CTRL2020_CONFIGPROVIDER_H

class ConfigProvider{
private:
    ConfigProvider(){}

    double steps_per_second = 15;
    //ToDo: Insert paramters for boundaries, etc.

    //----------------------------------Robot-Dimensions----------------------------------------------------------------

    double o = 115;
    double m = 330;
    double n = 645;
    double a = 1150;
    double b = 1220;

    //----------------------------------Joint-Boundaries----------------------------------------------------------------

    //Maximum Acceleration for all joints
    double a_max = 450;     // max acceleration

    //Theta 1
    double theta1_lower_border = (-1)*185;
    double theta1_upper_border = 185;
    double joint1_v_max = 120;     //max speed of joint 1 in °/s

    //Theta 2
    double theta2_lower_border = (-1)*140;
    double theta2_upper_border = (-1)*5;
    double joint2_v_max = 115;     //max speed of joint 2 in °/s

    //Theta 3
    double theta3_lower_border = (-1)*120;
    double theta3_upper_border = 168;
    double joint3_v_max = 120;     //max speed of joint 3 in °/s

    //Theta 4
    double theta4_lower_border = (-1)*350;
    double theta4_upper_border = 350;
    double joint4_v_max = 190;     //max speed of joint 4 in °/s

    //Theta 5
    double theta5_lower_border = (-1)*125;
    double theta5_upper_border = 125;
    double joint5_v_max = 180;     //max speed of joint 5 in °/s

    //Theta 6
    double theta6_lower_border = (-1)*350;
    double theta6_upper_border = 350;
    double joint6_v_max = 260;     //max speed of joint 6 in °/s

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

//-----------------------------------------------Singularity------------------------------------------------------------

    double margin_point = 0.2;
    // catch change of configuration type with change of joint angle
    double config_switch_limit = 90;



public:
    static ConfigProvider& getInstance(){
        static ConfigProvider instance;
        return instance;
    }
    ConfigProvider(ConfigProvider const&) = delete;
    void operator=(ConfigProvider const&) = delete;

    //ToDo: Insert Getter-Function for specific parameter --> Usage of ConfigProvider in specific class: ConfigProvider::getInstance().getParamA()


    double getsteps_per_second() const{return this->steps_per_second; }

    //Robot-Dimensions
    double geto() {return this-> o;}
    double getm() {return this-> m;}
    double getn() {return this-> n;}
    double geta() {return this-> a;}
    double getb() {return this-> b;}

    //Margin point for singularities
    double getmargin_point() const{return this->margin_point; }
    // Switch limit for configurations
    double getconfig_switch_limit() const{return this->config_switch_limit; }

    //Joint boundaries
    double getTheta1_lower_border(){return this->theta1_lower_border;}
    double getTheta1_upper_border(){return this->theta1_upper_border;}
    double getTheta2_lower_border(){return this->theta2_lower_border;}
    double getTheta2_upper_border(){return this->theta2_upper_border;}
    double getTheta3_lower_border(){return this->theta3_lower_border;}
    double getTheta3_upper_border(){return this->theta3_upper_border;}
    double getTheta4_lower_border(){return this->theta4_lower_border;}
    double getTheta4_upper_border(){return this->theta4_upper_border;}
    double getTheta5_lower_border(){return this->theta5_lower_border;}
    double getTheta5_upper_border(){return this->theta5_upper_border;}
    double getTheta6_lower_border(){return this->theta6_lower_border;}
    double getTheta6_upper_border(){return this->theta6_upper_border;}

    //Joint-Velocities
    double getJoint1_max_vel(){return this->joint1_v_max;}
    double getJoint2_max_vel(){return this->joint2_v_max;}
    double getJoint3_max_vel(){return this->joint3_v_max;}
    double getJoint4_max_vel(){return this->joint4_v_max;}
    double getJoint5_max_vel(){return this->joint5_v_max;}
    double getJoint6_max_vel(){return this->joint6_v_max;}
    double getMax_accel(){return this->a_max;}

};

#endif //SDIR_CTRL2020_CONFIGPROVIDER_H

