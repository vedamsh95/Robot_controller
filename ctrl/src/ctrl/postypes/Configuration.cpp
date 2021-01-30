#include "Configuration.h"

#include <cstring>
#include <iostream>

Json::Value* Configuration::serialize_to_json()
{
	Json::Value* json_pose = new Json::Value();
	(*json_pose)["j0"] = this->get_configuration()[0];
	(*json_pose)["j1"] = this->get_configuration()[1];
	(*json_pose)["j2"] = this->get_configuration()[2];
	(*json_pose)["j3"] = this->get_configuration()[3];
	(*json_pose)["j4"] = this->get_configuration()[4];
	(*json_pose)["j5"] = this->get_configuration()[5];

	return json_pose;
}

/*
void Configuration::deserialize_from_json(Json::Value _jv){
    this->set_configuration({_jv["j0"].asDouble(), _jv["j1"].asDouble(), _jv["j2"].asDouble(), _jv["j3"].asDouble(), _jv["j4"].asDouble(), _jv["j5"].asDouble()});
}
*/
void Configuration::deserialize_from_json(Json::Value _jv){
    double tmpTheta1 = 0.0;
    double tmpTheta2 = 0.0;
    double tmpTheta3 = 0.0;
    double tmpTheta4 = 0.0;
    double tmpTheta5 = 0.0;
    double tmpTheta6 = 0.0;

    //--------------------------------------------------check Theta 1 of Joint 1--------------------------------------//

    if( -185.0 > _jv["j0"].asDouble())
    {
        tmpTheta1 = -185.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j0"].asDouble() > 185.0)
    {
        tmpTheta1 = 185.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta1 = _jv["j0"].asDouble();

    //--------------------------------------------------check Theta 2 of Joint 2--------------------------------------//
    if( -140.0 > _jv["j1"].asDouble())
    {
        tmpTheta2 = -140.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j1"].asDouble() > -5.0)
    {
        tmpTheta2 = -5.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta2 = _jv["j1"].asDouble();

    //--------------------------------------------------check Theta 3 of Joint 3--------------------------------------//
    if( -120 > _jv["j2"].asDouble())
    {
        tmpTheta3 = -120.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j2"].asDouble() > 168.0)
    {
        tmpTheta3 = 168.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta3 = _jv["j2"].asDouble();

    //--------------------------------------------------check Theta 4 of Joint 4--------------------------------------//
    if( -350.0 > _jv["j3"].asDouble())
    {
        tmpTheta4 = -350.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j3"].asDouble() > 350.0)
    {
        tmpTheta4 = 350.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta4 = _jv["j3"].asDouble();


    //--------------------------------------------------check Theta 5 of Joint 5--------------------------------------//
    if( -125.0 > _jv["j4"].asDouble())
    {
        tmpTheta5 = -125.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j4"].asDouble() > 125.0)
    {
        tmpTheta5 = 125.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta5 = _jv["j4"].asDouble();

//--------------------------------------------------check Theta 6 of Joint 6--------------------------------------//
    if( -350.0 > _jv["j5"].asDouble())
    {
        tmpTheta6 = -350.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else if(_jv["j5"].asDouble() > 350.0)
    {
        tmpTheta6 = 350.0;
        std::cout<< "WARNING: Theta 0 not in range. Adaption executed automatically!" << std::endl;
    }
    else
        tmpTheta6 = _jv["j5"].asDouble();


    this->set_configuration({tmpTheta1,tmpTheta2,tmpTheta3,tmpTheta4,tmpTheta5,tmpTheta6});
}


Configuration::Configuration(Json::Value _jv){
    this->deserialize_from_json(_jv);
}




Configuration::Configuration() : joints{0,0,0,0,0,0}
{

}

Configuration::Configuration(std::array<double,NUM_JOINTS> _joints) : Configuration()
{
    this->set_configuration(_joints);
}

Configuration::Configuration(const Configuration& copy) : Configuration()
{
    this->set_configuration(copy.get_configuration());
}

void Configuration::set_configuration(const std::array<double,NUM_JOINTS> _joints)
{
    memcpy(this->joints.data(), _joints.data(), sizeof(double) * NUM_JOINTS);
}

/* Inline functions */

double& Configuration::operator[](size_t index)
{
    return this->joints[index];
}

const double Configuration::operator[](size_t index) const
{
    return this->joints[index];
}

std::array<double,NUM_JOINTS>& Configuration::get_configuration()
{
    return joints;
}

const std::array<double,NUM_JOINTS>& Configuration::get_configuration() const
{
    return joints;
}

void Configuration::operator=(const Configuration& copy)
{
    this->set_configuration( copy.get_configuration() );
}
