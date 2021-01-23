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

